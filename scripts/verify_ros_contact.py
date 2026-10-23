"""Verify contact ROS messages and services using an independent observer process."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--stop-during-scan',action='store_true')
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    if (output/'verification.json').exists() or (output/'execution').exists():
        parser.error('Existing verification output; select a new directory')
    os.environ.setdefault('ROS_DOMAIN_ID','48')
    env=os.environ.copy()
    env['PYTHONUNBUFFERED']='1'
    command=[sys.executable,'-m','desktop_arm.ros_contact','--output',str(output/'execution')]
    if args.config:
        command+=['--config',str(args.config.resolve())]
    rclpy.init()
    node=Node('contact_observer')
    forces=[]; states=[]; joints=[]
    node.create_subscription(WrenchStamped,'/contact/force',lambda m:forces.append({'time':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'force_z':m.wrench.force.z,'frame':m.header.frame_id}),100)
    node.create_subscription(String,'/contact/state',lambda m:states.append(m.data),100)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(list(m.position)),100)
    start=node.create_client(Trigger,'/contact/start')
    stop=node.create_client(Trigger,'/contact/stop')
    def call(client):
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('Service request timed out')
        return future.result().success
    began=time.monotonic()
    with (output/'server.log').open('w',encoding='utf-8') as log:
        server=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            if not start.wait_for_service(timeout_sec=35) or not stop.wait_for_service(timeout_sec=5):
                raise RuntimeError('Contact services unavailable')
            stop_before_start=call(stop)
            accepted=call(start)
            duplicate=call(start)
            stop_accepted=None
            deadline=time.monotonic()+150
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if args.stop_during_scan and stop_accepted is None and states.count('SCAN')>=10:
                    stop_accepted=call(stop)
                if server.poll() is not None:
                    break
            else:
                raise RuntimeError('Contact execution timed out')
            if server.returncode != 0:
                raise RuntimeError('Contact server failed; see server.log')
        finally:
            if server.poll() is None:
                server.terminate();server.wait(timeout=10)
            node.destroy_node();rclpy.shutdown()
    report=json.loads((output/'execution/summary.json').read_text(encoding='utf-8'))
    monotonic=all(b['time']>a['time'] for a,b in zip(forces,forces[1:]))
    good=accepted and not duplicate and not stop_before_start and len(forces)>100 and len(joints)>100 and monotonic and all(f['frame']=='world' for f in forces)
    if args.stop_during_scan:
        good=good and stop_accepted and report['controller_abort']=='external_stop' and report['final_force_n']<.1
    else:
        good=good and report['success']
    result=dict(passed=bool(good),start_accepted=accepted,duplicate_start_rejected=not duplicate,
                idle_stop_rejected=not stop_before_start,stop_during_scan=args.stop_during_scan,
                stop_accepted=stop_accepted,force_messages=len(forces),joint_messages=len(joints),
                phases=sorted(set(states)),timestamps_strictly_increasing=monotonic,
                elapsed_wall_seconds=time.monotonic()-began,execution=report)
    (output/'verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if forces:
        with (output/'received_force.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(forces[0]));writer.writeheader();writer.writerows(forces)
    print(json.dumps(result,indent=2),flush=True)
    if not good:
        raise SystemExit(2)


if __name__=='__main__':
    main()
