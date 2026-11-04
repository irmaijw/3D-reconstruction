import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import json
import numpy as np
import open3d as o3d
import cv2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

DEPTH_SCALE  = 1000.0     # uint16 depth PNGs store millimetres
# Image axes (+X right, +Y down, +Z forward) to Habitat's sensor axes
# (+X right, +Y up, +Z backward), used by the scorer.
CAM_TO_GT_AXES = np.diag([1.0, -1.0, -1.0])



def depth_image_to_point_cloud(rgb, depth_m, width, height, hfov):
    """Back-project one RGB-D frame into a colored 3-D point cloud.

    `rgb` is HxWx3 uint8 in BGR order and `depth_m` the same frame's HxW
    depths in metres; `width`/`height`/`hfov` are that capture's own camera
    parameters (read from its `intrinsics.json` by the caller — never
    hardcoded, so one capture is never unprojected through another's
    camera). Geometry is a pinhole with square pixels, image-centre
    principal point, and focal length derived from `width` and `hfov`.

    Returns points in metres in the frame's camera frame (+X right, +Y
    down, +Z forward) with colors as RGB floats in [0, 1]. A pixel yields
    a point iff `depth_m > 0` (zero means no return and is dropped, never
    placed at the origin).
    Points come out in row-major pixel order; an all-invalid frame yields
    an empty cloud. Inputs are not modified.
    """
    rgb = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.shape != (height, width) or rgb.shape != (height, width, 3):
        raise ValueError("RGB/depth dimensions must match the capture intrinsics")
    if width <= 0 or height <= 0 or not 0 < hfov < 180:
        raise ValueError("Invalid camera dimensions or horizontal FOV")
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    valid = np.isfinite(depth) & (depth > 0)
    z = depth[valid]
    focal = width / (2.0 * np.tan(np.deg2rad(hfov) / 2.0))
    points = np.column_stack(((u[valid] - width / 2.0) * z / focal,
                              (v[valid] - height / 2.0) * z / focal, z))
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(rgb[valid, ::-1] / 255.0)
    return cloud


def preprocess_point_cloud(pcd, voxel_size):
    """Downsample a cloud and prepare it for feature matching and ICP.

    Voxel-downsamples at `voxel_size`, estimates normals (radius
    `voxel_size * 2`, up to 30 neighbours) for point-to-plane ICP, and
    computes FPFH descriptors (radius `voxel_size * 5`, up to 100
    neighbours) for RANSAC matching. Returns the downsampled cloud and
    its FPFH feature set. Pairwise registration is performed by reconstruct().
    """
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("voxel_size must be positive and finite")
    down = pcd.voxel_down_sample(voxel_size)
    if not len(down.points):
        return down, o3d.pipelines.registration.Feature()
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_size * 2, max_nn=30))
    features = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 5, max_nn=100))
    return down, features


def local_icp_algorithm(source_down, target_down, trans_init, threshold):
    """Refine an initial alignment with point-to-plane ICP.

    Estimates normals on either cloud if missing (radius `threshold * 2`,
    up to 30 neighbours), then runs up to 100 ICP iterations at the single
    correspondence `threshold`. Returns the RegistrationResult; its
    `.transformation` is the refined 4x4.
    """
    if not np.isfinite(threshold) or threshold <= 0:
        raise ValueError("threshold must be positive and finite")
    reg = o3d.pipelines.registration
    if not len(source_down.points) or not len(target_down.points):
        return reg.evaluate_registration(source_down, target_down, threshold, trans_init)
    for cloud in (source_down, target_down):
        if not cloud.has_normals():
            cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
                radius=threshold * 2, max_nn=30))
    return reg.registration_icp(
        source_down, target_down, threshold, trans_init,
        reg.TransformationEstimationPointToPlane(),
        reg.ICPConvergenceCriteria(max_iteration=100))


class TemporalICP:
    """Multi-start Open3D ICP for sequential, overlapping capture frames.

    RANSAC is always attempted. Identity and the previous estimated relative
    motion supply additional initializations. Reject non-finite, zero-support,
    >1 m, or >60 degree updates. Among candidates within 0.01 of the highest
    fitness, minimize RMSE plus a small continuity penalty (0.005 per metre
    and per radian). No GT is used. The standard backend remains available
    for large viewpoint jumps where this capture-motion prior is unsuitable.
    """

    def __init__(self):
        self.previous = np.eye(4)
        self.diagnostics = {}

    @staticmethod
    def plausible(transform):
        return (np.isfinite(transform).all()
                and np.linalg.norm(transform[:3, 3]) <= 1.0
                and Rotation.from_matrix(transform[:3, :3].copy()).magnitude() <= np.pi / 3)

    def __call__(self, source, target, initial, threshold):
        candidates = []
        for name, guess in (("ransac", initial), ("identity", np.eye(4)),
                            ("previous", self.previous)):
            result = local_icp_algorithm(source, target, guess, threshold)
            if result.fitness > 0 and self.plausible(result.transformation):
                candidates.append((name, result))
        if candidates:
            best = max(result.fitness for _, result in candidates)
            shortlist = [(name, result) for name, result in candidates
                         if result.fitness >= best - 0.01]

            def cost(item):
                _, result = item
                change = np.linalg.inv(self.previous) @ result.transformation
                return (result.inlier_rmse + 0.005 * np.linalg.norm(change[:3, 3])
                        + 0.005 * Rotation.from_matrix(change[:3, :3]).magnitude())

            name, result = min(shortlist, key=cost)
        else:
            # Explicit motion-prior fallback, never a fabricated zero-error ICP.
            # Evaluate support at that transform and record the fallback source.
            name = "held_motion"
            result = o3d.pipelines.registration.evaluate_registration(
                source, target, threshold, self.previous)
        self.previous = result.transformation.copy()
        self.diagnostics = {"initialization_source": name,
                            "valid_icp_candidates": len(candidates),
                            "motion_prior_fallback": name == "held_motion"}
        return result


def my_local_icp_algorithm(source_down, target_down, trans_init, voxel_size):
    """From-scratch point-to-point ICP (the optional Bonus Track backend).

    Each iteration transforms the source by the current estimate, finds
    nearest neighbours in the target via cKDTree, keeps pairs closer than
    `voxel_size * 1.5` (stops below 10 survivors), and solves the optimal
    rigid update with the Kabsch SVD including a reflection fix. Composes
    each delta onto the running transform for up to 60 iterations, stopping
    when the mean inlier error changes by less than 1e-6. Returns a small
    object exposing the final 4x4 as `.transformation`.
    """
    # TODO: Initialize the running transform from `trans_init`.
    # TODO: Transform source points with the current estimate.
    # TODO: Establish nearest-neighbour correspondences against the target.
    # TODO: Reject correspondences outside an appropriate distance threshold
    # and stop safely when too few correspondences remain.
    # TODO: Use target normals to form and solve the point-to-plane
    # linearized pose update. Compose each update into the running 4x4
    # transform. Do not replace this with a call to Open3D's ICP; this is
    # the bonus custom backend selected by `version != "open3d"`.
    # TODO: Stop on convergence or a bounded iteration count.
    # TODO: Return a result-like object exposing the final transform as
    # `.transformation`.
    raise NotImplementedError("TODO: implement my_local_icp_algorithm")


def remove_ceiling_points(point_cloud, margin=1.0):
    """Return the map without the ceiling band, in frame-0 camera coordinates.

    Camera-frame +Y points down, so ceiling returns occupy the map's minimum
    Y values. Drop the same 1 m band that the viewer historically hid. This
    operates only on the merged output map; registration and trajectory scores
    continue to use the original per-frame depth clouds.
    """
    points = np.asarray(point_cloud.points)
    if len(points) == 0:
        return point_cloud
    keep = points[:, 1] > (points[:, 1].min() + float(margin))
    return point_cloud.select_by_index(np.flatnonzero(keep))


def _capture_files(data_root):
    """Map numeric PNG stems to paths; match identities rather than list positions."""
    def files(kind):
        directory = os.path.join(data_root, kind)
        return {int(os.path.splitext(name)[0]): os.path.join(directory, name)
                for name in os.listdir(directory)
                if name.lower().endswith(".png") and os.path.splitext(name)[0].isdigit()}
    return files("rgb"), files("depth")


def _evaluation_trajectories(predicted, data_root, frames):
    """GT in the first consumed camera frame, with explicit prediction row matches."""
    pred = np.asarray(predicted, dtype=np.float64).reshape(-1, 3) if predicted is not None else np.empty((0, 3))
    pred_disp = pred @ CAM_TO_GT_AXES
    empty = np.empty((0, 3))
    path = os.path.join(data_root, "GT_pose.npy") if data_root is not None else ""
    if not path or not os.path.isfile(path) or not len(pred):
        return pred_disp, empty, np.array([], dtype=int)
    gt = np.load(path, allow_pickle=False)
    if not gt.size:
        return pred_disp, empty, np.array([], dtype=int)
    if gt.ndim != 2 or gt.shape[1] != 7:
        raise ValueError("GT_pose.npy must have shape (N, 7): x,y,z,qw,qx,qy,qz")
    rgb, depth = _capture_files(data_root)
    ordered = sorted(rgb.keys() & depth.keys())
    requested = ordered if frames is None else [int(f) for f in frames]
    ordinals = {stem: i for i, stem in enumerate(ordered)}
    rows = [(i, ordinals[stem]) for i, stem in enumerate(requested[:len(pred)])
            if stem in ordinals and ordinals[stem] < len(gt)]
    # Anchor to the first consumed pose, never fit an alignment to predictions.
    anchor = ordinals.get(requested[0]) if requested else None
    if not rows or anchor is None or anchor >= len(gt):
        return pred_disp, empty, np.array([], dtype=int)
    pidx, gidx = np.asarray(rows, dtype=int).T
    q = gt[anchor, 3:7]
    rotation = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    gt_disp = (gt[gidx, :3] - gt[anchor, :3]) @ rotation
    return pred_disp, gt_disp, pidx


def _geometry_center(geoms):
    lower = np.min([g.get_min_bound() for g in geoms], axis=0)
    upper = np.max([g.get_max_bound() for g in geoms], axis=0)
    return (lower + upper) / 2.0


def _display_geometries(cloud, pred, gt):
    geoms = []
    if cloud is not None and not cloud.is_empty():
        display_cloud = (o3d.geometry.TriangleMesh(cloud)
                         if isinstance(cloud, o3d.geometry.TriangleMesh)
                         else o3d.geometry.PointCloud(cloud))
        axes = np.eye(4)
        axes[:3, :3] = CAM_TO_GT_AXES
        display_cloud.transform(axes)
        geoms.append(display_cloud)
    for trajectory, color in ((pred, [1., 0., 0.]), (gt, [0., 0., 0.])):
        if len(trajectory) < 2:
            continue
        edges = np.column_stack((np.arange(len(trajectory) - 1), np.arange(1, len(trajectory))))
        lines = o3d.geometry.LineSet()
        lines.points = o3d.utility.Vector3dVector(trajectory)
        lines.lines = o3d.utility.Vector2iVector(edges)
        lines.colors = o3d.utility.Vector3dVector(np.tile(color, (len(edges), 1)))
        geoms.append(lines)
    return geoms


# ══════════════════════════════════════════════════════════════════════════════
#  Reconstruction (headless) + metric
# ══════════════════════════════════════════════════════════════════════════════
def reconstruct(data_root, version="open3d", voxel_size=0.05, verbose=True,
                build_cloud=True, frames=None,
                down_voxel=None, registration_seed=0,
                pipeline_callback=None,
                diagnostics_callback=None, processed_frames=None,
                registration_profile="standard", camera_poses=None):
    """Reconstruct a camera trajectory and optionally merge its point clouds.

    Steps for each selected frame: build and preprocess an RGB-D cloud; register
    it to the previous frame with RANSAC initialization and ICP refinement; chain
    the transform and, when requested, add the cloud to the map.

    Frame selection:
      frames=None -> whole batch: every stem present under BOTH rgb/ and
        depth/, ascending numeric order.
      frames=[...] -> only those integer stems, IN THE GIVEN ORDER;
        "consecutive" means consecutive in the subset.

    Returns (global_pcd, pred_cam_pos):
      global_pcd   : every frame's cloud in the FRAME-0 CAMERA frame
                     (+X right, +Y down, +Z forward). Empty when
                     build_cloud=False (trajectory identical either way).
      pred_cam_pos : (N,3) float64 camera centres, frame-0 anchored.
                     GT stays on disk; visualize_and_evaluate loads and
                     subsets it by the same stems when scoring.
    If supplied, processed_frames is populated with the successfully consumed
    stems so callers can preserve GT correspondence when an image is unreadable.
    """
    if version not in {"open3d", "my_icp"}:
        raise ValueError("version must be open3d or my_icp")
    if registration_profile not in {"standard", "temporal"}:
        raise ValueError("registration_profile must be standard or temporal")
    if registration_profile != "standard" and version != "open3d":
        raise ValueError("temporal profile requires the Open3D ICP backend")
    temporal = TemporalICP() if registration_profile == "temporal" else None
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("voxel_size must be positive and finite")
    if down_voxel is not None and (not np.isfinite(down_voxel) or down_voxel <= 0):
        raise ValueError("down_voxel must be positive and finite")
    with open(os.path.join(data_root, "intrinsics.json"), encoding="utf-8") as handle:
        intrinsics = json.load(handle)
    width, height = int(intrinsics["width"]), int(intrinsics["height"])
    hfov = float(intrinsics["hfov"])
    scale = float(intrinsics.get("depth_scale", DEPTH_SCALE))
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("depth_scale must be positive and finite")
    rgb_files, depth_files = _capture_files(data_root)
    stems = sorted(rgb_files.keys() & depth_files.keys()) if frames is None else [int(f) for f in frames]
    missing = [stem for stem in stems if stem not in rgb_files or stem not in depth_files]
    if missing:
        raise ValueError(f"Missing RGB/depth pairs for frames {missing[:5]}")
    pairs = [(rgb_files[stem], depth_files[stem]) for stem in stems]
    o3d.utility.random.seed(int(registration_seed))
    reg = o3d.pipelines.registration

    T_global = np.eye(4)          # frame i -> frame-0 camera frame
    pred_cam_pos = []
    global_pcd = o3d.geometry.PointCloud()
    prev_down = prev_fpfh = None
    prev_depth_path = None
    prev_stem = None
    prev_pair_index = None
    icp_thr = voxel_size * 1.5

    for i, (rgb_path, depth_path) in enumerate(pairs):
        stem = stems[i]
        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        raw_depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        depth_m = None if raw_depth is None else raw_depth.astype(np.float64)
        if raw_depth is not None and np.issubdtype(raw_depth.dtype, np.integer):
            depth_m /= scale
        if rgb is None or depth_m is None:
            if verbose:
                print(f"  Warning: could not load frame {i}, skipping.")
            continue

        pcd = depth_image_to_point_cloud(rgb, depth_m, width, height, hfov)
        cur_down, cur_fpfh = preprocess_point_cloud(pcd, voxel_size)
        if pipeline_callback is not None:
            pipeline_callback("frame", {
                "stem": stem,
                "raw_point_count": len(pcd.points),
                "point_yield": len(cur_down.points),
            })

        if prev_down is not None:
            if len(cur_down.points) >= 4 and len(prev_down.points) >= 4:
                coarse_result = reg.registration_ransac_based_on_feature_matching(
                    cur_down, prev_down, cur_fpfh, prev_fpfh, True, icp_thr,
                    reg.TransformationEstimationPointToPoint(False), 3,
                    [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                     reg.CorrespondenceCheckerBasedOnDistance(icp_thr)],
                    reg.RANSACConvergenceCriteria(100000, 0.999))
            else:
                coarse_result = reg.evaluate_registration(
                    cur_down, prev_down, icp_thr, np.eye(4))
            trans_init = coarse_result.transformation
            if version == 'open3d':
                result_icp = (temporal if temporal is not None else local_icp_algorithm)(
                    cur_down, prev_down, trans_init, icp_thr)
            else:   # my_icp
                result_icp = my_local_icp_algorithm(
                    cur_down, prev_down, trans_init, voxel_size)
            T_global = T_global @ result_icp.transformation
            if pipeline_callback is not None and prev_pair_index == i - 1:
                pipeline_callback("pair", {
                    "prev_stem": prev_stem,
                    "stem": stem,
                    "coarse_fitness": float(coarse_result.fitness),
                    "coarse_rmse": float(coarse_result.inlier_rmse),
                    "coarse_correspondences": len(coarse_result.correspondence_set),
                    "icp_fitness": float(getattr(result_icp, "fitness", float("nan"))),
                    "icp_rmse": float(getattr(result_icp, "inlier_rmse", float("nan"))),
                    "icp_correspondences": len(getattr(result_icp, "correspondence_set", [])),
                })
            if diagnostics_callback is not None and prev_pair_index == i - 1:
                diagnostics_callback({
                    "frame_index": i,
                    "prev_stem": prev_stem,
                    "stem": stem,
                    "raw_point_count": len(pcd.points),
                    "point_yield": len(cur_down.points),
                    "coarse_fitness": float(coarse_result.fitness),
                    "coarse_rmse": float(coarse_result.inlier_rmse),
                    "coarse_correspondences": len(coarse_result.correspondence_set),
                    "icp_fitness": float(result_icp.fitness),
                    "icp_rmse": float(result_icp.inlier_rmse),
                    "icp_correspondences": len(getattr(result_icp, "correspondence_set", [])),
                    "trans_init": trans_init,
                    "icp_transformation": result_icp.transformation,
                    **(temporal.diagnostics if temporal is not None else
                       {"initialization_source": "ransac",
                        "motion_prior_fallback": False}),
                })

        pred_cam_pos.append(T_global[:3, 3].copy())
        if processed_frames is not None:
            processed_frames.append(stem)
        if camera_poses is not None:
            camera_poses.append(T_global.copy())

        if build_cloud:
            map_cloud = pcd.voxel_down_sample(down_voxel) if down_voxel else o3d.geometry.PointCloud(pcd)
            map_cloud.transform(T_global)
            global_pcd += map_cloud
            if down_voxel:
                global_pcd = global_pcd.voxel_down_sample(down_voxel)

        prev_down, prev_fpfh = cur_down, cur_fpfh
        prev_depth_path = depth_path
        prev_stem = stem
        prev_pair_index = i

        if verbose and (i % 25 == 0 or i == len(pairs) - 1):
            print(f"  frame {i:>4d}/{len(pairs)-1}")

    if not pred_cam_pos:
        return o3d.geometry.PointCloud(), np.zeros((0, 3))
    if build_cloud:
        global_pcd = remove_ceiling_points(global_pcd)
    return global_pcd, np.array(pred_cam_pos, dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════════════
#  Evaluation + visualisation (used by the thin reconstruct.py CLI)
# ══════════════════════════════════════════════════════════════════════════════
def visualize_and_evaluate(reconstructed_pcd, predicted_cam_poses, data_root,
                           frames=None, title="reconstruction", show=True,
                           save_path=None):
    """Score the trajectory and visualise map + trajectories.

    GT is loaded here from data_root/GT_pose.npy as (M,7)
    [x,y,z,qw,qx,qy,qz], or None when absent. GT rows are in capture order
    while image stems are identifiers, so when `frames` is given each stem
    is mapped to its ordinal in the sorted common RGB/depth frame list
    before indexing (stems absent from the capture or beyond GT are skipped,
    requested order preserved) — the score stays meaningful on subsets.

    1. Mean L2 over camera centres: reconcile frames first (pred is frame-0
       camera on image axes → flip Y and Z; GT world poses → R0^T (t_i - t_0)
       via pose 0), then mean over i of ||pred[i] - gt[i]||_2 with
       L2 = sqrt(dx^2 + dy^2 + dz^2). No scale/rotation/offset fitting TO the
       GT. inf when either side is missing/empty. Prints it and returns it.
    2. Estimated trajectory LineSet (red), GT trajectory LineSet (black).
    3. Open3D window with the ceiling-cropped map plus both trajectories
       (skipped when show=False, e.g. headless scoring). Ceiling points have
       already been removed from the merged map by `reconstruct`.
    """
    pred_disp, gt_disp, pred_indices = _evaluation_trajectories(
        predicted_cam_poses, data_root, frames)
    n = len(gt_disp)
    mean_l2_error = (float(np.linalg.norm(pred_disp[pred_indices] - gt_disp, axis=1).mean())
                     if n else float("inf"))
    print(f"Mean L2 distance: {mean_l2_error:.6f} meters  (over {n} frames, "
          f"{0 if predicted_cam_poses is None else len(predicted_cam_poses)} "
          f"reconstructed)")
    if save_path is not None:
        save_reconstruction_image(
            reconstructed_pcd, predicted_cam_poses, data_root,
            save_path, frames=frames, title=title)
    if not show:
        return mean_l2_error
    geoms = _display_geometries(reconstructed_pcd, pred_disp, gt_disp)
    if not geoms:
        return mean_l2_error
    print(f"[visualize] opening visualiser for {title} …  (press Q to close)")
    o3d.visualization.draw_geometries(
        geoms, window_name=f'HW1 Reconstruction - {title}',
        zoom=0.5, front=[0, 1, 0], lookat=_geometry_center(geoms), up=[1, 0, 0])
    return mean_l2_error

def save_reconstruction_image(reconstructed_pcd, predicted_cam_poses, data_root,
                              save_path, frames=None, title="reconstruction",
                              image_size=(1920, 1080)):
    """Render reconstructed point cloud and camera trajectories to a PNG file."""
    if save_path is None:
        return None
    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)

    pred_disp, gt_disp, _ = _evaluation_trajectories(predicted_cam_poses, data_root, frames)
    geoms = _display_geometries(reconstructed_pcd, pred_disp, gt_disp)
    if not os.environ.get("DISPLAY"):
        if os.path.exists("/tmp/.X11-unix/X1"):
            os.environ["DISPLAY"] = ":1"
        elif os.path.exists("/tmp/.X11-unix/X0"):
            os.environ["DISPLAY"] = ":0"

    rendered = False
    if geoms and os.environ.get("DISPLAY"):
        try:
            vis = o3d.visualization.Visualizer()
            if vis.create_window(visible=False, width=int(image_size[0]), height=int(image_size[1])):
                for g in geoms:
                    vis.add_geometry(g)
                opt = vis.get_render_option()
                if opt is not None:
                    opt.background_color = np.asarray([1.0, 1.0, 1.0])
                    if hasattr(opt, "line_width"):
                        opt.line_width = 3.0
                ctr = vis.get_view_control()
                if ctr is not None:
                    ctr.set_zoom(0.5)
                    # Display geometry is Y-up; +Y puts the camera above
                    # the map. A -Y front views the floor from underneath.
                    ctr.set_front([0, 1, 0])
                    ctr.set_lookat(_geometry_center(geoms))
                    ctr.set_up([1, 0, 0])
                vis.poll_events()
                vis.update_renderer()
                vis.capture_screen_image(save_path, do_render=True)
                vis.destroy_window()
                rendered = os.path.isfile(save_path) and os.path.getsize(save_path) > 0
        except Exception:
            rendered = False

    if not rendered:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection="3d")
            if reconstructed_pcd is not None and not reconstructed_pcd.is_empty():
                pts = np.asarray(reconstructed_pcd.vertices if isinstance(
                    reconstructed_pcd, o3d.geometry.TriangleMesh) else reconstructed_pcd.points)
                if len(pts) > 5000:
                    pts = pts[::len(pts) // 5000]
                pts_gt = pts @ CAM_TO_GT_AXES
                ax.scatter(pts_gt[:, 0], pts_gt[:, 1], pts_gt[:, 2], s=0.5, c="gray", alpha=0.3)
            if pred_disp is not None and len(pred_disp) > 0:
                ax.plot(pred_disp[:, 0], pred_disp[:, 1], pred_disp[:, 2], "r-", label="Predicted")
            if gt_disp is not None and len(gt_disp) > 0:
                ax.plot(gt_disp[:, 0], gt_disp[:, 1], gt_disp[:, 2], "k-", label="Ground Truth")
            ax.set_title(title)
            ax.legend()
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            plt.close(fig)
            rendered = os.path.isfile(save_path) and os.path.getsize(save_path) > 0
        except Exception as exc:
            print(f"[reconstruct] warning: failed to save reconstruction image {save_path}: {exc}")
    return save_path if rendered else None
