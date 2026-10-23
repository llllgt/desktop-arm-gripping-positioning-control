"""ROS 2 wrapper for the local 2 ms simulation loop and measured force stream."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .contact_process import ContactExperiment
from .force_control import ContactConfig


class ContactNode(Node):
    def __init__(self, config, output):
        super().__init__('contact_process')
        self.sim = ContactExperiment(config)
        self.output = output
        self.running = False
        self.finished_at = None
        self.index = 0
        self.force_pub = self.create_publisher(WrenchStamped, '/contact/force', 10)
        self.state_pub = self.create_publisher(String, '/contact/state', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_service(Trigger, '/contact/start', self.start)
        self.create_service(Trigger, '/contact/stop', self.stop)
        self.create_timer(config.timestep, self.tick)
        (output/'config.json').write_text(json.dumps(asdict(config),indent=2),encoding='utf-8')

    def start(self, request, response):
        response.success = not self.running and self.finished_at is None
        response.message = 'started' if response.success else 'already started or finished'
        if response.success:
            self.running = True
        return response

    def stop(self, request, response):
        response.success = self.running
        response.message = 'retracting' if self.running else 'not running'
        if self.running:
            self.sim.control.abort('external_stop', float(self.sim.data.time))
        return response

    def tick(self):
        if not self.running:
            return
        row = self.sim.step()
        if self.index % 5 == 0:
            stamp = float(row['time'])
            msg = WrenchStamped()
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
            if msg.header.stamp.nanosec >= 1000000000:
                msg.header.stamp.sec += 1
                msg.header.stamp.nanosec = 0
            msg.header.frame_id = 'world'
            msg.wrench.force.z = row['sensor_filtered']
            self.force_pub.publish(msg)
            self.state_pub.publish(String(data=row['phase']))
            joints = JointState()
            joints.header = msg.header
            joints.name = [f'joint{i}' for i in range(1,5)]+['probe_compression']
            joints.position = [row[f'q{i}'] for i in range(1,5)]+[row['compression_m']]
            self.joint_pub.publish(joints)
        self.index += 1
        if self.sim.control.phase == 'DONE':
            if self.finished_at is None:
                self.finished_at = float(self.sim.data.time)+.5
            if self.sim.data.time >= self.finished_at:
                self.running = False
                self.sim.save(self.output)
        limit = self.sim.c.approach_timeout+1.5+self.sim.c.scan_seconds+4
        if self.sim.data.time > limit:
            self.running = False
            self.finished_at = float(self.sim.data.time)
            self.sim.save(self.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output directory must be empty')
    c = ContactConfig(**json.loads(args.config.read_text(encoding='utf-8-sig'))) if args.config else ContactConfig()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node = ContactNode(c,args.output)
    try:
        deadline = time.monotonic()+180
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if node.finished_at is not None and not node.running:
                break
        else:
            raise RuntimeError('ROS process timeout')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
