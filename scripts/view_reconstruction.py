"""Inspect an existing reconstructed map without repeating registration."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import utils
import api
import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment', help='Experiment workspace or Turtle file')
    parser.add_argument('--mode', choices=['baseline', 'selected'], default='baseline')
    parser.add_argument('--no-vis', action='store_true', help='Recheck score without opening a window')
    parser.add_argument('--surface', action='store_true', help='View the estimated-pose TSDF surface mesh instead of raw merged points')
    args = parser.parse_args()
    path = Path(api.resolve_experiment_path(args.experiment))
    exp = api.read_experiment(str(path))
    directory = path.parent
    metadata = json.loads((directory/f'metrics_{args.mode}.json').read_text())
    trajectory = np.load(directory/f'trajectory_{args.mode}.npy', allow_pickle=False)
    cloud_path = directory/f'{"surface" if args.surface else "map"}_{args.mode}.ply'
    if not cloud_path.is_file():
        parser.error(f'Map not found: {cloud_path}')
    cloud = (o3d.io.read_triangle_mesh(str(cloud_path)) if args.surface
             else o3d.io.read_point_cloud(str(cloud_path)))
    count = len(cloud.vertices) if args.surface else len(cloud.points)
    print(f"Opening {args.mode}: {count:,} map vertices/points, {len(trajectory)} frames")
    score = utils.visualize_and_evaluate(
        cloud, trajectory, exp['batch_path'], frames=metadata['frame_stems'],
        title=f"{exp['exp_name']} ({args.mode})", show=not args.no_vis)
    if metadata['mean_l2_m'] is not None and not np.isclose(score, metadata['mean_l2_m'], atol=1e-8):
        raise ValueError('Recomputed score differs from saved metrics; inspect capture/trajectory provenance')


if __name__ == '__main__':
    main()
