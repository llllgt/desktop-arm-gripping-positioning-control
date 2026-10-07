"""ROS 2 actions drive MuJoCo through DDS, with JointState and pose feedback.

This is an original simulation action controller, not a ros2_control plugin.
It exposes the standard FollowJointTrajectory and GripperCommand interfaces.
"""
import argparse
import csv
import json
import threading
import time
from pathlib import Path
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration
from .kinematics import JOINT_NAMES, LIMITS, forward
from .simulation import Simulator
from .task import plan_task
from .paths import project_root


def validate_points(names,points):
    if tuple(names)!=JOINT_NAMES or len(points)<2:
        raise ValueError('Expected joint1..joint4 and at least two samples')
    times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points])
    positions=np.array([p.positions for p in points],dtype=float)
    if positions.shape!=(len(points),4) or not np.all(np.isfinite(positions)):
        raise ValueError('Invalid or nonfinite joint positions')
    if times[0]!=0 or np.any(np.diff(times)<=0):
        raise ValueError('Sample times must increase strictly')
    if np.any(positions<LIMITS[:,0]-1e-9) or np.any(positions>LIMITS[:,1]+1e-9):
        raise ValueError('Joint limit violation')
    if np.any(np.abs(np.diff(positions,axis=0)/np.diff(times)[:,None])>0.8001):
        raise ValueError('Command exceeds configured 0.8 rad/s speed limit')
    return times,positions


class SimulationController(Node):
    def __init__(self,output):
        super().__init__('desktop_arm_simulation')
        self.sim=Simulator()
        self.output=Path(output)
        self.output.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()
        self.command=self.sim.observation()['q']
        self.opening=self.sim.task['gripper_open']
        self.arm_busy=self.gripper_busy=False
        self.shutdown_requested=False
        self.shutdown_at=None
        self.phase='WAITING'
        self.rows=[]
        self.started=time.perf_counter()
        self.accepted_goals=0
        group=ReentrantCallbackGroup()
        self.joints=self.create_publisher(JointState,'/joint_states',10)
        self.object_pub=self.create_publisher(PoseStamped,'/desktop_arm/object_pose',10)
        self.tcp_pub=self.create_publisher(PoseStamped,'/desktop_arm/tcp_pose',10)
        self.create_subscription(String,'/desktop_arm/phase',self.set_phase,10,callback_group=group)
        self.create_service(Trigger,'/desktop_arm/save_results',self.save,callback_group=group)
        self.create_service(Trigger,'/desktop_arm/shutdown',self.shutdown,callback_group=group)
        self.arm_server=ActionServer(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',execute_callback=self.execute_arm,goal_callback=self.accept_arm,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.gripper_server=ActionServer(self,GripperCommand,'/gripper_controller/gripper_cmd',execute_callback=self.execute_gripper,goal_callback=self.accept_gripper,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.tick_count=0
        self.timer=self.create_timer(0.002,self.tick)
        self.get_logger().info('ROS controller ready: real MuJoCo contact simulation, standard action endpoints')

    def set_phase(self,msg):
        self.phase=msg.data

    def accept_arm(self,goal):
        try:
            validate_points(goal.trajectory.joint_names,goal.trajectory.points)
        except (ValueError,TypeError) as e:
            self.get_logger().error(str(e))
            return GoalResponse.REJECT
        with self.lock:
            if self.arm_busy:
                return GoalResponse.REJECT
            self.arm_busy=True
        return GoalResponse.ACCEPT

    def accept_gripper(self,goal):
        if not np.isfinite(goal.command.position) or not -.011<=goal.command.position<=.020 or not np.isfinite(goal.command.max_effort) or not 0<=goal.command.max_effort<=6:
            return GoalResponse.REJECT
        with self.lock:
            if self.gripper_busy:
                return GoalResponse.REJECT
            self.gripper_busy=True
        return GoalResponse.ACCEPT

    def execute_arm(self,handle):
        times,positions=validate_points(handle.request.trajectory.joint_names,handle.request.trajectory.points)
        with self.lock:
            start=self.sim.data.time
            self.accepted_goals+=1
        result=FollowJointTrajectory.Result()
        try:
            while rclpy.ok():
                with self.lock:
                    elapsed=self.sim.data.time-start
                    self.command=np.array([np.interp(elapsed,times,positions[:,i]) for i in range(4)])
                    obs=self.sim.observation()
                if handle.is_cancel_requested:
                    with self.lock:
                        self.command=obs['q']
                    handle.canceled()
                    result.error_code=FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string='Canceled; controller holds current measured position'
                    return result
                feedback=FollowJointTrajectory.Feedback()
                feedback.joint_names=list(JOINT_NAMES)
                feedback.actual.positions=obs['q'].tolist()
                feedback.desired.positions=self.command.tolist()
                feedback.error.positions=(self.command-obs['q']).tolist()
                handle.publish_feedback(feedback)
                if elapsed>=times[-1]:
                    break
                time.sleep(.01)
            handle.succeed()
            result.error_code=FollowJointTrajectory.Result.SUCCESSFUL
            return result
        finally:
            with self.lock:
                self.arm_busy=False

    def execute_gripper(self,handle):
        with self.lock:
            start=self.sim.data.time
            original=self.opening
        result=GripperCommand.Result()
        while rclpy.ok():
            with self.lock:
                elapsed=self.sim.data.time-start
                u=float(np.clip(elapsed/.8,0,1))
                s=10*u**3-15*u**4+6*u**5
                self.opening=original+s*(handle.request.command.position-original)
                # Honour the requested total effort within the 6 N design cap.
                effort=handle.request.command.max_effort or 6.0
                self.sim.model.actuator_forcerange[4]=[-effort,effort]
                result.position=float(self.sim.data.qpos[self.sim.model.joint('gripper_left_joint').qposadr[0]])
                result.effort=float(abs(self.sim.data.actuator_force[4]))
            if handle.is_cancel_requested:
                with self.lock:
                    self.opening=result.position
                    self.gripper_busy=False
                handle.canceled()
                return result
            if elapsed>=.8:
                break
            time.sleep(.01)
        result.reached_goal=abs(result.position-handle.request.command.position)<.0005
        result.stalled=not result.reached_goal
        with self.lock:
            self.gripper_busy=False
        handle.succeed()
        return result

    def tick(self):
        with self.lock:
            self.sim.step(self.command,self.opening)
            if self.tick_count%5==0:
                obs=self.sim.observation()
                row={'time':float(self.sim.data.time),'phase':self.phase}
                for i in range(4):
                    row[f'q{i+1}']=obs['q'][i]
                    row[f'q{i+1}_command']=self.command[i]
                    row[f'dq{i+1}']=obs['dq'][i]
                    row[f'tau{i+1}']=self.sim.data.actuator_force[i]
                for i,a in enumerate('xyz'):
                    row['object_'+a]=obs['object'][i]
                    row['tcp_'+a]=obs['tcp'][i]
                    row['target_'+a]=forward(self.command).position[i]
                self.rows.append(row)
                stamp=self.get_clock().now().to_msg()
                msg=JointState()
                msg.header.stamp=stamp
                msg.name=[*JOINT_NAMES,'gripper_left_joint','gripper_right_joint']
                msg.position=[*obs['q'],float(self.sim.data.qpos[self.sim.model.joint('gripper_left_joint').qposadr[0]]),float(self.sim.data.qpos[self.sim.model.joint('gripper_right_joint').qposadr[0]])]
                msg.velocity=[*obs['dq'],0.0,0.0]
                self.joints.publish(msg)
                for pub,key in [(self.object_pub,'object'),(self.tcp_pub,'tcp')]:
                    p=PoseStamped()
                    p.header.stamp=stamp;p.header.frame_id='world'
                    p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,obs[key])
                    quat=np.zeros(4)
                    matrix=(self.sim.data.xmat[self.sim.object_body] if key=='object'
                            else self.sim.data.site_xmat[self.sim.tcp_site])
                    mujoco.mju_mat2Quat(quat,matrix)
                    p.pose.orientation.w,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z=map(float,quat)
                    pub.publish(p)
            self.tick_count+=1

    def save(self,request,response):
        with self.lock:
            rows=list(self.rows)
            final=self.sim.observation()['object']
            task=self.sim.task
        with (self.output/'trajectory.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        actual=np.array([[r[f'q{i}'] for i in range(1,5)] for r in rows])
        target=np.array([[r[f'q{i}_command'] for i in range(1,5)] for r in rows])
        distance=float(np.linalg.norm(final-task['place']))
        lift=float(max(r['object_z'] for r in rows)-task['pick'][2])
        summary={'execution':'Two ROS 2 nodes, standard trajectory/gripper actions over DDS, MuJoCo contact physics',
            'arm_action_goals':self.accepted_goals,'joint_state_samples':len(rows),
            'simulation_seconds':self.sim.data.time,'wall_seconds':time.perf_counter()-self.started,
            'joint_rmse_rad':np.sqrt(np.mean((actual-target)**2,axis=0)).tolist(),
            'placement_error_mm':distance*1000,'maximum_lift_mm':lift*1000,
            'final_workpiece_xyz_m':final.tolist(),'success':bool(distance<.008 and lift>.024),
            'object_attachment_used':False}
        (self.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        response.success=True;response.message=json.dumps(summary)
        return response

    def shutdown(self,request,response):
        self.shutdown_at=time.monotonic()+.5
        response.success=True;response.message='Controller shutdown requested'
        return response


class TransferClient(Node):
    def __init__(self):
        super().__init__('desktop_arm_transfer')
        self.arm=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        self.gripper=ActionClient(self,GripperCommand,'/gripper_controller/gripper_cmd')
        self.phase_pub=self.create_publisher(String,'/desktop_arm/phase',10)
        self.save_client=self.create_client(Trigger,'/desktop_arm/save_results')
        self.stop_client=self.create_client(Trigger,'/desktop_arm/shutdown')

    def wait(self,future,timeout=60):
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def execute(self):
        if not self.arm.wait_for_server(timeout_sec=20) or not self.gripper.wait_for_server(timeout_sec=20):
            raise RuntimeError('Controller action server unavailable')
        for phase in plan_task():
            msg=String();msg.data=phase.name;self.phase_pub.publish(msg)
            self.get_logger().info('Executing '+phase.name)
            goal=FollowJointTrajectory.Goal()
            goal.trajectory.joint_names=list(JOINT_NAMES)
            for t in np.linspace(0,phase.segment.duration,int(np.ceil(phase.segment.duration/.02))+1):
                sample=phase.segment.sample(t)
                point=JointTrajectoryPoint()
                point.positions=sample.position.tolist();point.velocities=sample.velocity.tolist();point.accelerations=sample.acceleration.tolist()
                ns=round(t*1e9);point.time_from_start=Duration(sec=ns//10**9,nanosec=ns%10**9)
                goal.trajectory.points.append(point)
            arm_handle=self.wait(self.arm.send_goal_async(goal))
            if not arm_handle.accepted:
                raise RuntimeError('Arm goal rejected')
            grip_handle=None
            if phase.gripper_end!=phase.gripper_start:
                g=GripperCommand.Goal();g.command.position=phase.gripper_end;g.command.max_effort=6.0
                grip_handle=self.wait(self.gripper.send_goal_async(g))
                if not grip_handle.accepted:
                    raise RuntimeError('Gripper goal rejected')
            result=self.wait(arm_handle.get_result_async())
            if result.result.error_code!=FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(result.result.error_string)
            if grip_handle is not None:
                self.wait(grip_handle.get_result_async())
        if not self.save_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Result service unavailable')
        saved=self.wait(self.save_client.call_async(Trigger.Request()))
        print(saved.message,flush=True)
        if not saved.success or not json.loads(saved.message)['success']:
            raise RuntimeError('Physics placement checks failed')
        self.stop_client.wait_for_service(timeout_sec=5)
        self.wait(self.stop_client.call_async(Trigger.Request()))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['server','task'])
    parser.add_argument('--output',type=Path,default=project_root()/'results/ros2')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    if args.mode=='server':
        node=SimulationController(args.output)
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
        try:
            while rclpy.ok() and (node.shutdown_at is None or time.monotonic()<node.shutdown_at):
                executor.spin_once(timeout_sec=.1)
        finally:
            executor.shutdown();node.destroy_node();rclpy.shutdown()
    else:
        node=TransferClient()
        try:
            node.execute()
        finally:
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
