"""Fuse RGB-D frames using saved ESTIMATED camera poses into a colored TSDF mesh.

This changes map presentation, never trajectory estimates or evaluation scores.
The assignment's NumPy depth-unprojection implementation remains unchanged.
"""
import os
os.environ.setdefault('OMP_NUM_THREADS','1')
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import utils
import cv2
import numpy as np
import open3d as o3d


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',required=True)
    parser.add_argument('--camera-poses',required=True,help='Estimated (N,4,4) camera-to-first-frame poses; never GT_pose.npy')
    parser.add_argument('--output',required=True,help='Output mesh .ply')
    parser.add_argument('--metrics',help='Run metrics JSON containing exact frame_stems')
    parser.add_argument('--voxel-size',type=float,default=.02)
    parser.add_argument('--cut-y',type=float,default=0.,help='Visualization crop: keep geometry below initial camera height, image-frame +Y down')
    args=parser.parse_args()
    if not np.isfinite(args.voxel_size) or args.voxel_size<=0:parser.error('voxel-size must be positive and finite')
    if Path(args.camera_poses).name=='GT_pose.npy':parser.error('Use estimated camera matrices, not ground truth')
    capture=Path(args.capture);poses=np.load(args.camera_poses,allow_pickle=False)
    rgb,depth=utils._capture_files(capture)
    stems=(json.loads(Path(args.metrics).read_text())['frame_stems'] if args.metrics else sorted(rgb.keys()&depth.keys()))
    if poses.shape!=(len(stems),4,4) or not np.isfinite(poses).all():parser.error('Estimated poses must have shape (frame_count,4,4) and be finite')
    intr=json.loads((capture/'intrinsics.json').read_text());w,h=int(intr['width']),int(intr['height'])
    focal=w/(2*np.tan(np.deg2rad(intr['hfov'])/2))
    camera=o3d.camera.PinholeCameraIntrinsic(w,h,focal,focal,w/2,h/2)
    volume=o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel_size,sdf_trunc=args.voxel_size*4,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    start=time.perf_counter()
    for i,(stem,pose) in enumerate(zip(stems,poses)):
        color=cv2.cvtColor(cv2.imread(rgb[stem]),cv2.COLOR_BGR2RGB)
        raw=cv2.imread(depth[stem],cv2.IMREAD_UNCHANGED)
        metres=raw.astype(np.float32)
        if np.issubdtype(raw.dtype,np.integer):metres/=float(intr.get('depth_scale',1000))
        rgbd=o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color),o3d.geometry.Image(metres),depth_scale=1.,
            depth_trunc=10.,convert_rgb_to_intensity=False)
        volume.integrate(rgbd,camera,np.linalg.inv(pose))
        if i%50==0:print('integrated',i,'/',len(stems),flush=True)
    mesh=volume.extract_triangle_mesh()
    mesh.remove_vertices_by_mask(np.asarray(mesh.vertices)[:,1]<args.cut_y)
    mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(output),mesh):raise RuntimeError('Failed to save mesh')
    metadata={'capture':str(capture.resolve()),'estimated_poses':str(Path(args.camera_poses).resolve()),
              'estimated_poses_sha256':hashlib.sha256(Path(args.camera_poses).read_bytes()).hexdigest(),
              'frame_count':len(stems),'voxel_size_m':args.voxel_size,'sdf_trunc_m':args.voxel_size*4,
              'cut_y_m':args.cut_y,'depth_trunc_m':10.,'vertices':len(mesh.vertices),
              'triangles':len(mesh.triangles),'seconds':time.perf_counter()-start,
              'note':'Estimated-pose TSDF fusion and visualization crop only; no GT, no trajectory correction.'}
    output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('MESH_SAVED',output,metadata['triangles'],'triangles',flush=True)


if __name__=='__main__':main()
