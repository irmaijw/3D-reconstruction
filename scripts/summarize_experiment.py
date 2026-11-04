"""Export report tables and static factor plots from an existing experiment."""
import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import api
import plot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment', help='Experiment directory or Turtle file')
    args = parser.parse_args()
    path = api.resolve_experiment_path(args.experiment)
    exp, _, factors, _, frame_count, _, _ = plot._experiment_data(path)
    if not exp['complete']:
        raise ValueError('Incomplete pipeline evidence; refusing to export a report summary')
    workspace = Path(path).parent
    output = workspace/'plots'
    output.mkdir(exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(14, 13), constrained_layout=True)
    for ax, (name, records) in zip(axes.flat, factors.items()):
        rows = sorted(records, key=lambda r: r['frame'])
        ax.plot([r['frame'] for r in rows],
                [float('nan') if r['value'] is None else r['value'] for r in rows],
                linewidth=1)
        ax.set_title(name)
        ax.set_xlabel('Current frame identifier')
        ax.set_ylabel('RMSE (m)' if 'RMSE' in name else 'Fitness' if 'Fitness' in name else 'Count')
        ax.grid(alpha=.25)
    fig.suptitle(f"{exp['exp_name']} — recorded baseline pipeline measurements")
    fig.savefig(output/'factors_static.png', dpi=160)
    plt.close(fig)

    fields = ['mode','frame_count','retained_fraction','mean_l2_m',
              'reconstruction_seconds','total_execution_seconds','voxel_size_m',
              'map_voxel_size_m','registration_seed','icp_backend','selection_query']
    with (workspace/'run_comparison.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for mode in ['baseline', 'selected']:
            metric_path = workspace/f'metrics_{mode}.json'
            if metric_path.exists():
                row = json.loads(metric_path.read_text())
                row['retained_fraction'] = row['frame_count']/frame_count
                writer.writerow(row)
    print('Validated seal and',sum(exp['factor_counts'].values()),'factor records')
    print('Saved',output/'factors_static.png')
    print('Saved',workspace/'run_comparison.csv')


if __name__ == '__main__':
    main()
