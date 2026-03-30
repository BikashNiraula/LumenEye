#!/usr/bin/env python3
"""
Fake Joint Executor — RViz mode.
Subscribes to /arm_controller/joint_trajectory and smoothly interpolates
joint positions, publishing to /joint_states so RViz shows the movement.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time

JOINT_NAMES = [
    'lss_arm_joint_1',
    'lss_arm_joint_2',
    'lss_arm_joint_3',
    'lss_arm_joint_4',
    'lss_arm_joint_5',
    'lss_arm_joint_6',
]

class FakeJointExecutor(Node):
    def __init__(self):
        super().__init__('fake_joint_executor')

        self.current = {j: 0.0 for j in JOINT_NAMES}
        self.target  = {j: 0.0 for j in JOINT_NAMES}
        self.duration_sec = 3.0
        self.start_pos = {j: 0.0 for j in JOINT_NAMES}
        self.move_start = None
        self.moving = False

        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._traj_cb, 10)

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info('Fake Joint Executor ready (RViz mode)')

    def _traj_cb(self, msg):
        if not msg.points:
            return
        point = msg.points[-1]  # take final target point
        for i, name in enumerate(msg.joint_names):
            if name in self.target and i < len(point.positions):
                self.target[name] = point.positions[i]
        # duration from trajectory point
        self.duration_sec = max(
            point.time_from_start.sec +
            point.time_from_start.nanosec / 1e9, 0.5)
        self.start_pos = dict(self.current)
        self.move_start = time.time()
        self.moving = True
        self.get_logger().info(
            f'Moving to: { {k: round(v,3) for k,v in self.target.items()} }')

    def _tick(self):
        if self.moving and self.move_start is not None:
            elapsed = time.time() - self.move_start
            t = min(elapsed / self.duration_sec, 1.0)
            # smooth step interpolation
            t_smooth = t * t * (3 - 2 * t)
            for j in JOINT_NAMES:
                self.current[j] = (self.start_pos[j] +
                    t_smooth * (self.target[j] - self.start_pos[j]))
            if t >= 1.0:
                self.moving = False
                self.get_logger().info('Movement complete')

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [self.current[j] for j in JOINT_NAMES]
        js.velocity = [0.0] * len(JOINT_NAMES)
        js.effort   = [0.0] * len(JOINT_NAMES)
        self.js_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    n = FakeJointExecutor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()