"""Exercise validation and cancellation using actual DDS action requests."""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from desktop_arm.ros_nodes import TransferClient, validate_points
from desktop_arm.task import plan_task
from desktop_arm.kinematics import JOINT_NAMES


def check():
    q=plan_task()[0].segment.start.tolist()
    def goal(seconds=1):
        g=FollowJointTrajectory.Goal();g.trajectory.joint_names=list(JOINT_NAMES)
        for sec in (0,seconds):
            p=JointTrajectoryPoint();p.positions=q;p.time_from_start=Duration(sec=sec)
            g.trajectory.points.append(p)
        return g
    g=goal()
    validate_points(g.trajectory.joint_names,g.trajectory.points)
    local_cases=0
    for mutate in (
        lambda v:setattr(v.trajectory,'joint_names',['bad']),
        lambda v:setattr(v.trajectory.points[-1],'time_from_start',Duration()),
        lambda v:setattr(v.trajectory.points[-1],'positions',[float('nan')]*4),
        lambda v:setattr(v.trajectory.points[-1],'positions',[20.]*4),
        lambda v:setattr(v.trajectory.points[-1],'positions',[q[0]+1,q[1],q[2],q[3]]),
        lambda v:setattr(v.trajectory.points[0],'time_from_start',Duration(sec=1)),
    ):
        invalid=goal();mutate(invalid)
        try:
            validate_points(invalid.trajectory.joint_names,invalid.trajectory.points)
        except ValueError:
            local_cases+=1
        else:
            raise AssertionError('Malformed trajectory accepted')
    node=TransferClient()
    try:
        assert node.arm.wait_for_server(timeout_sec=20)
        assert node.gripper.wait_for_server(timeout_sec=20)
        invalid=goal();invalid.trajectory.joint_names=['bad']
        assert not node.wait(node.arm.send_goal_async(invalid)).accepted
        grip=GripperCommand.Goal();grip.command.position=1.;grip.command.max_effort=6.
        assert not node.wait(node.gripper.send_goal_async(grip)).accepted
        handle=node.wait(node.arm.send_goal_async(goal(10)))
        assert handle.accepted
        assert not node.wait(node.arm.send_goal_async(goal())).accepted
        cancel=node.wait(handle.cancel_goal_async())
        assert cancel.goals_canceling
        result=node.wait(handle.get_result_async())
        assert result.status==GoalStatus.STATUS_CANCELED
        report={'local_invalid_cases_rejected':local_cases,'dds_invalid_arm_rejected':True,
                'dds_invalid_gripper_rejected':True,'dds_overlapping_arm_rejected':True,
                'dds_cancel_status':result.status,'success':True}
        root=Path(__file__).resolve().parents[1]
        (root/'results/ros2/action_checks.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
    finally:
        node.destroy_node()


if __name__=='__main__':
    rclpy.init()
    try:
        check()
    finally:
        rclpy.shutdown()
