import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
import open3d as o3d
import pytest
from scipy.spatial.transform import Rotation
import utils


def capture(tmp_path, stems=(1, 4, 9)):
    (tmp_path/'rgb').mkdir(); (tmp_path/'depth').mkdir()
    (tmp_path/'intrinsics.json').write_text(json.dumps(dict(width=4,height=4,hfov=90,depth_scale=2000)))
    for stem in stems:
        cv2.imwrite(str(tmp_path/'rgb'/f'{stem}.png'),np.full((4,4,3),128,np.uint8))
        cv2.imwrite(str(tmp_path/'depth'/f'{stem}.png'),np.full((4,4),2000,np.uint16))
    return tmp_path


def test_unprojection_units_colors_invalid_and_order():
    rgb=np.array([[[0,0,255],[255,0,0]],[[0,255,0],[255,255,255]]],np.uint8)
    depth=np.array([[2.,0.],[np.nan,4.]])
    cloud=utils.depth_image_to_point_cloud(rgb,depth,2,2,90.)
    np.testing.assert_allclose(cloud.points,[[-2,-2,2],[0,0,4]],atol=1e-12)
    np.testing.assert_allclose(cloud.colors,[[1,0,0],[1,1,1]])
    assert np.isnan(depth[1,0]) and depth[0,0]==2
    assert len(utils.depth_image_to_point_cloud(rgb,np.zeros((2,2)),2,2,90).points)==0
    with pytest.raises(ValueError): utils.depth_image_to_point_cloud(rgb,depth,3,2,90)


def test_icp_recovers_known_rigid_transform():
    rng=np.random.default_rng(5)
    pts=rng.normal(size=(1200,3)); pts/=np.linalg.norm(pts,axis=1)[:,None]
    pts*=np.array([1.,.7,.4])
    target=o3d.geometry.PointCloud(); target.points=o3d.utility.Vector3dVector(pts)
    truth=np.eye(4); truth[:3,:3]=Rotation.from_rotvec([.015,-.02,.01]).as_matrix(); truth[:3,3]=[.01,-.015,.02]
    source=o3d.geometry.PointCloud(target); source.transform(np.linalg.inv(truth))
    result=utils.local_icp_algorithm(source,target,np.eye(4),.15)
    np.testing.assert_allclose(result.transformation,truth,atol=2e-3)
    down,feature=utils.preprocess_point_cloud(target,.06)
    assert down.has_normals() and feature.data.shape==(33,len(down.points))


def test_noncontiguous_subset_gt_anchor_rotation_and_association(tmp_path):
    capture(tmp_path)
    rot=Rotation.from_euler('z',90,degrees=True)
    gt=np.zeros((3,7)); gt[:,:3]=[[5,7,2],[5,8,2],[5,10,2]]
    q=rot.as_quat(); gt[:,3:]=q[[3,0,1,2]]
    np.save(tmp_path/'GT_pose.npy',gt)
    assert utils.visualize_and_evaluate(None,[[0,0,0],[2,0,0]],tmp_path,frames=[4,9],show=False)<1e-12
    # Missing stem must not shift the association of a later prediction.
    assert utils.visualize_and_evaluate(None,[[0,0,0],[99,99,99],[2,0,0]],tmp_path,frames=[4,88,9],show=False)<1e-12
    assert np.isinf(utils.visualize_and_evaluate(None,[],tmp_path,show=False))
    (tmp_path/'GT_pose.npy').unlink()
    assert np.isinf(utils.visualize_and_evaluate(None,[[0,0,0]],tmp_path,show=False))


def test_transform_composition_depth_scale_callbacks_and_map_independence(tmp_path,monkeypatch):
    capture(tmp_path)
    transforms=[]
    first=np.eye(4); first[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix(); first[:3,3]=[1,0,0]
    second=np.eye(4); second[:3,3]=[1,0,0]
    seen=[]
    def icp(*args):
        seen.append(np.asarray(args[0].points)[:,2].copy())
        r=o3d.pipelines.registration.RegistrationResult(); r.transformation=[first,second][len(transforms)%2]; transforms.append(r.transformation)
        return r
    monkeypatch.setattr(utils,'local_icp_algorithm',icp)
    records=[]
    _,pred=utils.reconstruct(tmp_path,verbose=False,build_cloud=False,pipeline_callback=lambda kind,val:records.append((kind,val)))
    np.testing.assert_allclose(pred,[[0,0,0],[1,0,0],[1,1,0]],atol=1e-12)
    assert len([r for r in records if r[0]=='frame'])==3
    assert [(r[1]['prev_stem'],r[1]['stem']) for r in records if r[0]=='pair']==[(1,4),(4,9)]
    assert all(np.allclose(z,1) for z in seen)
    _,with_map=utils.reconstruct(tmp_path,verbose=False,build_cloud=True,down_voxel=.1)
    np.testing.assert_allclose(with_map,pred)


def test_empty_depth_still_records_all_factors(tmp_path):
    capture(tmp_path,stems=(1,4))
    for p in (tmp_path/'depth').glob('*.png'): cv2.imwrite(str(p),np.zeros((4,4),np.uint16))
    records=[]
    cloud,pred=utils.reconstruct(tmp_path,verbose=False,pipeline_callback=lambda k,v:records.append((k,v)))
    assert pred.shape==(2,3) and len(cloud.points)==0
    assert len(records)==3


def test_scoring_camera_axes(tmp_path):
    capture(tmp_path,stems=(1,4))
    np.save(tmp_path/'GT_pose.npy',np.array([[10,20,30,1,0,0,0],[12,17,26,1,0,0,0]],float))
    assert utils.visualize_and_evaluate(None,[[0,0,0],[2,3,4]],tmp_path,show=False)<1e-12


def test_unreadable_frame_cannot_silently_shift_gt(tmp_path):
    capture(tmp_path,stems=(1,4))
    (tmp_path/'rgb'/'1.png').write_bytes(b'broken PNG')
    consumed=[]
    _,pred=utils.reconstruct(tmp_path,verbose=False,build_cloud=False,processed_frames=consumed)
    assert consumed==[4] and pred.shape==(1,3)


def test_explicit_frame_order(tmp_path):
    capture(tmp_path)
    records=[]
    _,pred=utils.reconstruct(tmp_path,frames=[9,1],verbose=False,build_cloud=False,
        pipeline_callback=lambda k,v:records.append((k,v)))
    assert [v['stem'] for k,v in records if k=='frame']==[9,1]
    assert [(v['prev_stem'],v['stem']) for k,v in records if k=='pair']==[(9,1)]
    assert pred.shape==(2,3)


def test_temporal_rejects_implausible_high_fitness_motion(monkeypatch):
    bad=np.eye(4);bad[0,3]=1000
    good=np.eye(4);good[0,3]=.2
    def fake(source,target,guess,threshold):
        result=o3d.pipelines.registration.RegistrationResult()
        result.transformation=bad if np.array_equal(guess,bad) else good
        result.fitness=1. if np.array_equal(guess,bad) else .9
        result.inlier_rmse=.01
        return result
    monkeypatch.setattr(utils,'local_icp_algorithm',fake)
    tracker=utils.TemporalICP()
    result=tracker(None,None,bad,.075)
    np.testing.assert_allclose(result.transformation,good)
    assert tracker.diagnostics['valid_icp_candidates']==2
    assert not tracker.diagnostics['motion_prior_fallback']


def test_temporal_zero_support_uses_explicit_evaluated_fallback(monkeypatch):
    def failed(*args):return o3d.pipelines.registration.RegistrationResult()
    monkeypatch.setattr(utils,'local_icp_algorithm',failed)
    tracker=utils.TemporalICP()
    cloud=o3d.geometry.PointCloud();cloud.points=o3d.utility.Vector3dVector([[0.,0.,1.]])
    result=tracker(cloud,cloud,np.eye(4),.075)
    np.testing.assert_allclose(result.transformation,np.eye(4))
    assert tracker.diagnostics['motion_prior_fallback']
    assert len(result.correspondence_set)==1
    assert result.fitness==1.
    # Every reconstruction starts a fresh tracker; no state leaks to another run.
    tracker.previous[0,3]=.2
    np.testing.assert_allclose(utils.TemporalICP().previous,np.eye(4))


def test_camera_pose_export_matches_trajectory(tmp_path):
    capture(tmp_path,stems=(1,))
    poses=[]
    _,trajectory=utils.reconstruct(tmp_path,build_cloud=False,verbose=False,camera_poses=poses)
    assert np.asarray(poses).shape==(1,4,4)
    np.testing.assert_allclose(np.asarray(poses)[:,:3,3],trajectory)
