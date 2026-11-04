"""Thin CLI over utils.py (reconstruct.py).

    Full module guide migrated to docs/reconstruct.md — baseline +
    selected runs, query-driven selection, write-back, and CLI flags.
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import sys
import argparse
import shutil
import json
import time
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import api                                              # noqa: E402
import utils                                            # noqa: E402


def _segments_from_frames(frames):
    """Sorted frame indices -> maximal contiguous segments, including singletons."""
    segments = []
    for frame in sorted(set(int(value) for value in frames)):
        if segments and frame == segments[-1][-1] + 1:
            segments[-1].append(frame)
        else:
            segments.append([frame])
    return segments


def _plan_query_selected(exp, query_path):
    """SPARQL file -> (frames, segments) to reconstruct.

    Runs the query against the experiment graph via
    api.frames_from_sparql_query, records maximal contiguous segments for
    gap metadata, then reconstructs the flattened sequence across those gaps.
    Returns (None, []) when the selection is empty (nothing to run).
    """
    try:
        selected = api.frames_from_sparql_query(exp, query_path)
    except OSError as exc:
        raise ValueError(f"cannot read query {query_path!r}: {exc}") from None
    segments = _segments_from_frames(selected)
    if not segments:
        print("[reconstruct] SPARQL selection is EMPTY: no selected run.")
        return None, []
    frames = [frame for segment in segments for frame in segment]
    spans = ", ".join(f"{segment[0]}..{segment[-1]}({len(segment)})"
                      for segment in segments[:8])
    if len(segments) > 8:
        spans += f", … +{len(segments) - 8} more"
    print(f"[reconstruct] SPARQL selection: {len(segments)} segment(s), "
          f"{len(frames)} frames [{spans}]")
    return frames, segments



def _segment_metadata(segments):
    """Statusless evidence for the temporal discontinuities in a selected run."""
    if not segments:
        return {"spliceCount": 0, "maxGapLength": 0}
    gaps = [max(0, int(right[0]) - int(left[-1]) - 1)
            for left, right in zip(segments, segments[1:])]
    return {"spliceCount": max(0, len(segments) - 1),
            "maxGapLength": max(gaps, default=0)}


def _save_artifacts(exp_dir, mode, cloud, trajectory, error, args, data_root,
                    reconstruction_seconds, total_seconds, processed_frames, camera_poses=None, diagnostics=None):
    """Keep reproducibility and report evidence outside the sealed RDF graph."""
    np.save(os.path.join(exp_dir, f"trajectory_{mode}.npy"), trajectory)
    if len(cloud.points):
        o3d.io.write_point_cloud(os.path.join(exp_dir, f"map_{mode}.ply"), cloud)
    if camera_poses is not None:
        np.save(os.path.join(exp_dir, f"camera_poses_{mode}.npy"), np.asarray(camera_poses))
    if diagnostics is not None:
        with open(os.path.join(exp_dir, f"registration_diagnostics_{mode}.json"), "w", encoding="utf-8") as handle:
            json.dump(diagnostics, handle, indent=2, default=lambda x: x.tolist(), allow_nan=False)
    payload = {
        "registration_profile": args.registration_profile,
        "temporal_motion_limits": {"translation_m": 1.0, "rotation_deg": 60, "fitness_slack": 0.01, "continuity_weight": 0.005} if args.registration_profile == "temporal" else None,
        "mode": mode, "capture": os.path.abspath(data_root),
        "frame_count": len(trajectory), "frame_stems": processed_frames,
        "mean_l2_m": error if np.isfinite(error) else None,
        "reconstruction_seconds": reconstruction_seconds,
        "total_execution_seconds": total_seconds,
        "voxel_size_m": args.voxel_size, "map_voxel_size_m": args.map_voxel_size,
        "registration_seed": args.seed,
        "icp_backend": args.version or "open3d",
        "ransac_max_iterations": 100000, "ransac_confidence": 0.999,
        "icp_max_iterations": 100, "correspondence_distance_m": args.voxel_size * 1.5,
        "selection_query": args.selection_query,
    }
    with open(os.path.join(exp_dir, f"metrics_{mode}.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, allow_nan=False)
        handle.write("\n")


def main():
    """Workflow: declare -> reconstruct (measures + seals) -> select -> reconstruct -> show.

    1. ``api.py declare`` scaffolds the experiment and reconstruction settings.
    2. First-pass ``reconstruct.py --experiment DECL.ttl`` measures the raw data
       inline and seals the machine section.
    3. Here, frame selection is one of two modes:
       fullbatch (frames=None, the whole batch) or a SPARQL file
       (``--selection-query``) whose ?frame/?frameIndex bindings become the
       frame list fed to ``utils.reconstruct``.
    4. One reconstruction run, one Mean L2 number, one Open3D window.
    """
    parser = argparse.ArgumentParser(
        description="Reconstruct a capture (fullbatch or SPARQL-selected frames), "
                    "print Mean L2, show the cloud + trajectories.")
    # dest default is None ON PURPOSE: it is the only way to tell "caller asked for
    # open3d" from "caller said nothing", and the icpBackend rule below needs that.
    parser.add_argument('-v', '--version', type=str, default=None,
                        help='open3d or my_icp (default: the experiment\'s icpBackend '
                             'setting, else open3d). Passing this AND an experiment '
                             'that records a different backend is an error.')
    parser.add_argument('--voxel-size', type=float, default=0.05,
                        help='registration voxel size in metres (default: 0.05)')
    parser.add_argument('--seed', type=int, default=0,
                        help='Open3D RANSAC seed (default: 0)')
    parser.add_argument('--no-vis', action='store_true',
                        help='skip the Open3D window (print metrics only)')
    parser.add_argument('--experiment', type=str, required=True,
                        help='experiment Turtle (or its workspace directory); an '
                             'UNSEALED declaration triggers the measuring first pass, '
                             'which consumes the declared hw1:batchFile. The run value '
                             'is written back below the machine marker.')
    parser.add_argument('--selection-query', metavar='QUERY.rq', default=None,
                        help='SPARQL SELECT over --experiment binding ?frame (frame IRI) '
                             'or ?frameIndex (integer). The bindings become the frame '
                             'list; results are cut into contiguous segments.')
    parser.add_argument('--override-experiment', '--write', dest='override_experiment',
                        action='store_true', default=False,
                        help='write run results to the experiment file (default: no-write / dry run)')
    parser.add_argument('--no-write', action='store_true', default=False,
                        help='explicit dry run (default behavior): print the results but write nothing into the '
                             'experiment file')
    parser.add_argument('--map-voxel-size', type=float, default=0.025,
                        help='merged-map voxel size in metres; trajectory is unaffected (default: 0.025)')
    parser.add_argument('--registration-profile', choices=['standard', 'temporal'], default=None,
                        help='must match the experiment setting; default: recorded profile or standard')
    args = parser.parse_args()
    started = time.perf_counter()

    args.experiment = api.resolve_experiment_path(args.experiment)
    if args.selection_query and not os.path.isfile(args.selection_query):
        candidate = os.path.join(api._workspace_for_experiment_file(args.experiment),
                                 "queries", args.selection_query)
        if os.path.isfile(candidate):
            args.selection_query = candidate

    if not np.isfinite(args.map_voxel_size) or args.map_voxel_size <= 0:
        parser.error("--map-voxel-size must be positive and finite")
    if not np.isfinite(args.voxel_size) or args.voxel_size <= 0:
        parser.error("--voxel-size must be positive")
    if args.seed < 0:
        parser.error("--seed must be non-negative")
    version = args.version

    exp = None
    if not api.is_assessed(args.experiment):
        # Unsealed declaration: first-pass measuring baseline run
        if args.selection_query is not None:
            parser.error("an unsealed declaration is measured by a full-batch baseline run; selection runs come after sealing")
        if args.no_write:
            parser.error("the measuring run writes the machine section; drop --no-write")

        decl = api.read_declaration(args.experiment)
        # No --data_root: the measuring run consumes the declared hw1:batchFile.
        data_root = decl["batch_path"]

        set_backend = decl["required"].get("icpBackend")
        if set_backend is not None:
            set_backend = str(set_backend)
            if version is None:
                version = set_backend
                print(f"[reconstruct] icpBackend from declaration: {version}")
            elif version != set_backend:
                parser.error(
                    f"--version {version!r} disagrees with the icpBackend setting "
                    f"{set_backend!r} recorded in {args.experiment}. That setting is "
                    f"part of the treatment this experiment declares; write a new "
                    f"declaration for the other backend instead of overriding it.")
        if version is None:
            version = "open3d"

        recorded_profile = str(decl["required"].get("registrationProfile", "standard"))
        if args.registration_profile is not None and args.registration_profile != recorded_profile:
            parser.error("--registration-profile disagrees with the declaration; create a new experiment")
        args.registration_profile = recorded_profile
        frames = api._pair_frames(data_root)
        pipe_cb, pipe_frame_recs, pipe_pair_recs = api.make_pipeline_measurement_callback(
            api.load_quality_factors())

        build_cloud = True
        print("[reconstruct] === baseline measuring run ===")
        processed_frames = []
        camera_poses, diagnostics = [], []
        reconstruction_started = time.perf_counter()
        args.version = version
        pcd, pred_cam_pos = utils.reconstruct(
            data_root, version, frames=[int(stem) for stem, _, _ in frames],
            build_cloud=build_cloud, voxel_size=args.voxel_size,
            registration_seed=args.seed, pipeline_callback=pipe_cb,
            down_voxel=args.map_voxel_size, processed_frames=processed_frames,
            registration_profile=args.registration_profile, camera_poses=camera_poses,
            diagnostics_callback=diagnostics.append)
        reconstruction_seconds = time.perf_counter() - reconstruction_started

        # Require one set of frame factors per consumed frame and one set of
        # pair factors per adjacent consumed frame. An incomplete pipeline run
        # must never be sealed as if it had complete evidence.
        frame_stems = [int(stem) for stem, _, _ in frames]
        missing_keys = []
        for stem in frame_stems:
            for factor in ("RawPointCount", "VoxelizedPointYield"):
                if (factor, stem) not in pipe_frame_recs:
                    missing_keys.append(f"frame {stem} factor {factor}")
        for previous, current in zip(frame_stems, frame_stems[1:]):
            for factor in ("CoarseRANSACFitness", "CoarseRANSACRMSE",
                           "CoarseRANSACCorrespondences", "ICPInlierFitness",
                           "ICPInlierRMSE", "ICPInlierCorrespondences"):
                if (factor, previous, current) not in pipe_pair_recs:
                    missing_keys.append(
                        f"pair ({previous}, {current}) factor {factor}")
        if missing_keys:
            sample = ", ".join(missing_keys[:5])
            raise RuntimeError(
                f"measuring run failed to record all pipeline factors "
                f"({len(missing_keys)} missing); first missing: {sample}")

        exp_dir = api._workspace_for_experiment_file(args.experiment)
        png_path = os.path.join(exp_dir, "reconstruction.png")
        l2 = utils.visualize_and_evaluate(
            pcd, pred_cam_pos, data_root, frames=frame_stems,
            title=f'{os.path.basename(os.path.normpath(data_root))} ({version}, baseline)',
            show=not args.no_vis,
            save_path=png_path)
        if not os.path.isfile(png_path):
            utils.save_reconstruction_image(
                pcd, pred_cam_pos, data_root, png_path,
                frames=frame_stems,
                title=f'{os.path.basename(os.path.normpath(data_root))} ({version}, baseline)')
        if os.path.isfile(png_path):
            baseline_png = os.path.join(exp_dir, "reconstruction_baseline.png")
            if not os.path.isfile(baseline_png):
                shutil.copyfile(png_path, baseline_png)
            print(f"[reconstruct] saved reconstruction image -> {png_path}")

        _save_artifacts(exp_dir, "baseline", pcd, pred_cam_pos, l2, args, data_root,
                        reconstruction_seconds, time.perf_counter() - started, processed_frames,
                        camera_poses=camera_poses, diagnostics=diagnostics)
        api.seal_experiment(
            args.experiment, decl, frames,
            pipeline_frame_records=pipe_frame_recs,
            pipeline_pair_records=pipe_pair_recs)
        frame_count = len(pred_cam_pos)
        api.write_run(
            args.experiment, "baseline", {"mapMeanL2": l2},
            frame_count=frame_count)
        print(f"[reconstruct] wrote baseline run: Mean L2 Distance = {l2!r} "
              f"(stored as hw1:mapMeanL2 at file precision), "
              f"runFrameCount = {frame_count} -> {args.experiment}")
        return

    # Sealed run: consume the sealed hw1:batchPath (no --data_root to mismatch).
    exp = api.read_experiment(args.experiment)
    data_root = exp["batch_path"]
    if not data_root:
        parser.error(f"{args.experiment}: sealed batch carries no hw1:batchPath; re-seal from its declaration")
    print(f"[reconstruct] {args.experiment}: experiment {exp['exp_name']} on batch "
          f"{exp['batch_name']!r}, {len(exp['frame_indices'])} frames, "
          f"{sum(exp['factor_counts'].values())} factor occurrences")

    # The RECORDED icpBackend setting is authoritative.
    set_backend = exp["settings"].get("icpBackend")
    if set_backend is not None:
        set_backend = str(set_backend)
        if version is None:
            version = set_backend
            print(f"[reconstruct] icpBackend from experiment: {version}")
        elif version != set_backend:
            parser.error(
                f"--version {version!r} disagrees with the icpBackend setting "
                f"{set_backend!r} recorded in {args.experiment}. That setting is "
                f"part of the treatment this experiment declares; write a new "
                f"declaration for the other backend instead of overriding it.")
    if version is None:
        version = 'open3d'

    recorded_profile = str(exp["settings"].get("registrationProfile", "standard"))
    if args.registration_profile is not None and args.registration_profile != recorded_profile:
        parser.error("--registration-profile disagrees with the sealed experiment; create a new experiment")
    args.registration_profile = recorded_profile

    # Selection: fullbatch (whole batch) or SPARQL (frame list from query).
    if args.selection_query is not None:
        frames, segments = _plan_query_selected(exp, args.selection_query)
        if frames is None:
            return  # empty selection: nothing to reconstruct, nothing written
        mode = "selected"
    else:
        frames, segments = None, []
        mode = "baseline"

    # The map is only ever looked at by the visualiser; skipping it halves peak
    # memory and utils.reconstruct guarantees the trajectory is identical either way.
    build_cloud = True

    print(f"[reconstruct] === {mode} run ===")
    processed_frames = []
    camera_poses, diagnostics = [], []
    reconstruction_started = time.perf_counter()
    args.version = version
    pcd, pred_cam_pos = utils.reconstruct(
        data_root, version, frames=frames, build_cloud=build_cloud,
        voxel_size=args.voxel_size, registration_seed=args.seed,
        down_voxel=args.map_voxel_size, processed_frames=processed_frames,
        registration_profile=args.registration_profile, camera_poses=camera_poses,
        diagnostics_callback=diagnostics.append)
    reconstruction_seconds = time.perf_counter() - reconstruction_started
    exp_dir = api._workspace_for_experiment_file(args.experiment)
    # A preview must not overwrite the artifacts of an already recorded run.
    writes_run = args.override_experiment and not args.no_write
    artifact_mode = mode if writes_run else f"{mode}_preview"
    png_path = os.path.join(exp_dir, "reconstruction.png" if writes_run
                            else f"reconstruction_{artifact_mode}.png")
    l2 = utils.visualize_and_evaluate(
        pcd, pred_cam_pos, data_root, frames=processed_frames,
        title=f'{os.path.basename(os.path.normpath(data_root))} '
              f'({version}, {mode})',
        show=not args.no_vis,
        save_path=png_path)
    if png_path is not None:
        if not os.path.isfile(png_path):
            utils.save_reconstruction_image(
                pcd, pred_cam_pos, data_root, png_path,
                frames=processed_frames,
                title=f'{os.path.basename(os.path.normpath(data_root))} ({version}, {mode})')
        if os.path.isfile(png_path):
            mode_png = os.path.join(exp_dir, f"reconstruction_{artifact_mode}.png")
            if mode_png != png_path:
                shutil.copyfile(png_path, mode_png)
            print(f"[reconstruct] saved reconstruction image -> {png_path}")

    _save_artifacts(exp_dir, artifact_mode, pcd, pred_cam_pos, l2, args, data_root,
                    reconstruction_seconds, time.perf_counter() - started, processed_frames,
                        camera_poses=camera_poses, diagnostics=diagnostics)
    if not args.override_experiment or args.no_write:
        print(f"[reconstruct] default no-write (pass --override-experiment to write): {mode} Mean L2 Distance = {l2!r} NOT written "
              f"to {args.experiment}")
    else:
        frame_count = len(pred_cam_pos)
        if frames is not None and frame_count != len(frames):
            print(f"[reconstruct] *** WARNING: {mode} run selected {len(frames)} "
                  f"frames but reconstructed {frame_count}; some frame failed to "
                  f"load. hw1:runFrameCount reports what was consumed.",
                  file=sys.stderr)
        run_metadata = {}
        if mode == "selected":
            run_metadata.update(_segment_metadata(_segments_from_frames(processed_frames)))
        api.write_run(
            args.experiment, mode, {"mapMeanL2": l2},
            used_frames=processed_frames if mode == "selected" else None,
            frame_count=frame_count, metadata=run_metadata)
        print(f"[reconstruct] wrote {mode} run: Mean L2 Distance = {l2!r} "
              f"(stored as hw1:mapMeanL2 at file precision), "
              f"runFrameCount = {frame_count} -> {args.experiment}")


if __name__ == '__main__':
    main()
