"""Compare saved baseline/selected errors on the same retained frame identities."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import utils
import api
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('experiment')
    args=parser.parse_args()
    path=Path(api.resolve_experiment_path(args.experiment)); exp=api.read_experiment(str(path)); root=path.parent
    b=json.loads((root/'metrics_baseline.json').read_text());s=json.loads((root/'metrics_selected.json').read_text())
    frames=s['frame_stems']
    if not frames or frames[0]!=b['frame_stems'][0]:
        raise ValueError('This comparison requires the same first-camera anchor; a different anchor needs full estimated orientations')
    lookup={frame:i for i,frame in enumerate(b['frame_stems'])}
    bp=np.load(root/'trajectory_baseline.npy',allow_pickle=False)[[lookup[f] for f in frames]]
    sp=np.load(root/'trajectory_selected.npy',allow_pickle=False)
    baseline=utils.visualize_and_evaluate(None,bp,exp['batch_path'],frames=frames,show=False)
    selected=utils.visualize_and_evaluate(None,sp,exp['batch_path'],frames=frames,show=False)
    if not np.isclose(selected,s['mean_l2_m'],atol=1e-8):raise ValueError('Selected score no longer matches the saved metrics')
    payload={'frame_count':len(frames),'frame_stems':frames,
             'baseline_common_mean_l2_m':baseline,'selected_common_mean_l2_m':selected,
             'baseline_full_mean_l2_m':b['mean_l2_m'],
             'same_first_camera_anchor':True,
             'note':'Evaluation only; GT is not used for registration or selection.'}
    output=root/'metrics_common_frames.json';output.write_text(json.dumps(payload,indent=2)+'\n')
    print('Saved',output)
    print('Common-frame baseline / selected Mean L2:',baseline,selected)


if __name__=='__main__':main()
