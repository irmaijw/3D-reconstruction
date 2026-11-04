"""Check that interactive capture saves the updated camera view, not stale pixels."""
from pathlib import Path
import sys
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
import load
import simulator


@pytest.mark.parametrize('auto,blocked,manual,expected',[(True,False,False,1),(True,True,False,0),(False,False,True,1)])
def test_capture_after_pose_change(tmp_path,monkeypatch,auto,blocked,manual,expected):
    position=np.array([0.,0.,0.])
    def state():
        return S(sensor_states={'color_sensor':S(position=position.copy(),rotation=S(w=1.,x=0.,y=0.,z=0.))})
    def step(action):
        if not blocked: position[2]-=.2
    engine=S(agent=S(get_state=state),step=step,observe=lambda t:{'position':position.copy()})
    first=[S(type=2,key=10)]
    if manual:first.append(S(type=2,key=14))
    batches=iter([first,[S(type=2,key=16)]])
    pg=S(K_w=10,K_s=11,K_a=12,K_d=13,K_c=14,K_SPACE=15,K_q=16,K_ESCAPE=17,
         QUIT=1,KEYDOWN=2,event=S(get=lambda:next(batches)),
         time=S(Clock=lambda:S(tick=lambda _:None,get_fps=lambda:30)),
         key=S(set_repeat=lambda *args:None))
    saved=[]
    def save(frame,sensor,*args):
        np.testing.assert_allclose(frame['position'],sensor.position)
        saved.append(sensor.position.copy())
        return np.r_[sensor.position,[1.,0.,0.,0.]]
    monkeypatch.setattr(simulator,'save_frame',save)
    load.run_interactive(engine,S(pygame=pg,EXPOSE_EVENTS=()),S(draw=lambda *a,**k:None),30,str(tmp_path),{},auto_capture=auto)
    assert len(saved)==expected
    if expected:np.testing.assert_allclose(saved[0],[0,0,-.2])
    assert len(np.load(tmp_path/'GT_pose.npy'))==expected
