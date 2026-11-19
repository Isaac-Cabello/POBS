"""Simulation-only horizontal thrust safety gate (no sonar integration yet).

Starts disarmed, publishes zero thrust when disarmed/inactive/stale/tilted.
Do not use as a physical-hardware safety controller or independent watchdog.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry


class SimThrusterSafety(Node):
    def __init__(self):
        super().__init__('sim_thruster_safety')

        self.declare_parameter('enabled', False)
        self.declare_parameter('max_force_n', 1.0)
        self.declare_parameter('gazebo_namespace', 'pobs')

        namespace = self.get_parameter('gazebo_namespace').value
        prefix = f'/model/{namespace}/joint'

        self.pub4 = self.create_publisher(
            Float64, f'{prefix}/thruster_4_joint/cmd_thrust', 10
        )
        self.pub5 = self.create_publisher(
            Float64, f'{prefix}/thruster_5_joint/cmd_thrust', 10
        )
        self.status_pub = self.create_publisher(
            String, '/planner/gazebo_thruster_gate/status', 10
        )

        self.cmd = {}
        self.cmd_time = {}
        self.pose_time = 0.0
        self.planner_time = 0.0
        self.controller_time = 0.0
        self.pitch = None
        self.roll = None
        self.xy = None
        self.planner_state = ''
        self.controller_state = ''

        for index in (4, 5):
            self.create_subscription(
                Float64, f'/planner/dry_run/thruster_{index}',
                lambda msg, i=index: self.command_cb(i, msg), 10
            )
        self.create_subscription(
            Odometry, '/planner/sim_odometry',
            self.odom_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/planner/status', self.planner_cb, 10
        )
        self.create_subscription(
            String, '/planner/xy_controller/status',
            self.controller_cb, 10
        )
        self.create_timer(0.05, self.update)
        self.get_logger().info(
            'Simulation thruster safety started - DISARMED'
        )

    def command_cb(self, index, msg):
        self.cmd[index] = msg.data
        self.cmd_time[index] = time.monotonic()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y)
        if not all(math.isfinite(v) for v in values):
            return

        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.pitch = math.degrees(
            math.asin(max(-1.0, min(1.0, sinp)))
        )
        sinr = 2 * (q.w*q.x + q.y*q.z)
        cosr = 1 - 2 * (q.x*q.x + q.y*q.y)
        self.roll = math.degrees(math.atan2(sinr, cosr))
        self.xy = (p.x, p.y)
        self.pose_time = time.monotonic()

    def planner_cb(self, msg):
        try:
            self.planner_state = json.loads(msg.data).get('state', '')
        except (ValueError, TypeError, AttributeError):
            self.planner_state = ''
        self.planner_time = time.monotonic()

    def controller_cb(self, msg):
        try:
            self.controller_state = json.loads(msg.data).get('state', '')
        except (ValueError, TypeError, AttributeError):
            self.controller_state = ''
        self.controller_time = time.monotonic()

    def update(self):
        now = time.monotonic()
        state = 'SAFE_STOP'
        force4 = 0.0
        force5 = 0.0

        fresh = (
            self.pose_time > 0
            and now - self.pose_time < 0.5
            and now - self.planner_time < 0.5
            and now - self.controller_time < 0.5
            and all(
                i in self.cmd
                and now - self.cmd_time.get(i, 0) < 0.5
                and math.isfinite(self.cmd[i])
                for i in (4, 5)
            )
        )
        enabled = self.get_parameter('enabled').value

        if not enabled:
            state = 'DISARMED'
        elif not fresh:
            state = 'STALE_DATA'
        elif (
            abs(self.pitch) > 10.0
            or abs(self.roll) > 10.0
        ):
            state = 'UNSAFE_TILT'
        elif math.hypot(*self.xy) > 3.0:
            state = 'OUTSIDE_TEST_AREA'
        elif (
            self.planner_state != 'NAVIGATING'
            or self.controller_state != 'TRACKING'
        ):
            state = 'INACTIVE'
        else:
            state = 'ACTIVE'
            max_force = max(0.0, min(
                float(self.get_parameter('max_force_n').value), 2.0
            ))
            force4 = max(-1.0, min(1.0, self.cmd[4])) * max_force
            force5 = max(-1.0, min(1.0, self.cmd[5])) * max_force

        self.pub4.publish(Float64(data=float(force4)))
        self.pub5.publish(Float64(data=float(force5)))
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'pitch_deg': self.pitch,
            'roll_deg': self.roll,
            'force_4_n': force4,
            'force_5_n': force5,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = SimThrusterSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
