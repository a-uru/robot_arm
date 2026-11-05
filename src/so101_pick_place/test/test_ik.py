import xml.etree.ElementTree as ET
import numpy as np
from so101_servo.kinematics import Chain
from so101_pick_place.ik import tcp_candidates

DESCRIPTION = '''<robot name="test"><link name="base"/><link name="link"/><link name="tip"/>
<joint name="a" type="revolute"><parent link="base"/><child link="link"/><axis xyz="0 0 1"/></joint>
<joint name="b" type="revolute"><parent link="link"/><child link="tip"/><origin xyz="0.2 0 0"/><axis xyz="0 0 1"/></joint></robot>'''


def test_tcp_ik_includes_rotated_offset_and_joint_bounds():
    chain=Chain(DESCRIPTION,'base','tip')
    target,_=chain.evaluate({'a':.4,'b':-.7},[.1,0,0])
    candidate=next(tcp_candidates(chain,{'a':0.,'b':0.},[.1,0,0],target,{'a':(-1.,1.),'b':(-1.,1.)},.001,2.))
    point,_=chain.evaluate(candidate,[.1,0,0])
    assert np.linalg.norm(point-target)<.0005
    assert all(-1<=v<=1 for v in candidate.values())


def test_unreachable_tcp_returns_no_solution():
    chain=Chain(DESCRIPTION,'base','tip')
    result=list(tcp_candidates(chain,{'a':0.,'b':0.},[.1,0,0],np.array([1.,1.,1.]),
                               {'a':(-1.,1.),'b':(-1.,1.)},.001,.2))
    assert result == []
