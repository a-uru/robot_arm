#!/usr/bin/env python3
"""Read live held controller state and decompose TCP error against a logged target."""
import argparse
import ast,json,re,time
from pathlib import Path
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from control_msgs.msg import JointTrajectoryControllerState
from so101_servo.kinematics import Chain
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pick-log', required=True, help='Pick-place node log containing the frozen target')
parser.add_argument('--output', default='log/arm_tracking/tcp_error_snapshot')
args = parser.parse_args()
rclpy.init();n=rclpy.create_node('inspect_tcp_error')
c=n.create_client(GetParameters,'/so101_pick_place/get_parameters')
if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Pick node unavailable')
q=GetParameters.Request();q.names=['robot_description','base_frame','tip_link','tcp_offset_m']
f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
v=f.result().values
chain=Chain(v[0].string_value,v[1].string_value,v[2].string_value);offset=list(v[3].double_array_value)
messages=[]
sub=n.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',messages.append,10)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
if not messages:raise RuntimeError('No controller feedback')
m=messages[-1];ref=dict(zip(m.joint_names,m.reference.positions));actual=dict(zip(m.joint_names,m.feedback.positions))
p_ref,jac=chain.evaluate(ref,offset);p_actual,_=chain.evaluate(actual,offset)
log=Path(args.pick_log).read_text()
target=np.array(ast.literal_eval(re.findall(r'PLANNING: frozen target (\[[^\]]+\])',log)[-1]))
contributions={j:(jac[:,i]*(actual[j]-ref[j])*1000).tolist() for i,j in enumerate(chain.names)}
result={'controller_stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'target_m':target.tolist(),'tcp_offset_m':offset,'reference_tcp_m':p_ref.tolist(),'actual_tcp_m':p_actual.tolist(),'actual_minus_target_mm':((p_actual-target)*1000).tolist(),'reference_minus_target_mm':((p_ref-target)*1000).tolist(),'actual_minus_reference_mm':((p_actual-p_ref)*1000).tolist(),'distance_mm':float(np.linalg.norm(p_actual-target)*1000),'joint_actual_minus_reference_rad':{j:actual[j]-ref[j] for j in chain.names},'linearized_joint_contributions_mm':contributions,'linearization_remainder_mm':((p_actual-p_ref)*1000-sum(np.array(x) for x in contributions.values())).tolist(),'note':'Live sample after abort; original instant not recorded. Frozen target read from specified failure log.'}
result['reference_joints_rad'] = ref
result['actual_joints_rad'] = actual
p=Path(args.output);p.mkdir(parents=True,exist_ok=True);(p/'robot_description.xml').write_text(v[0].string_value);(p/'breakdown.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
n.destroy_node();rclpy.shutdown()
