import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, Bool


class MasterPlanner(Node):
    """Supervisory XY waypoint planner. Never commands thrusters directly."""

    def __init__(self):
        super().__init__('master_planner')
        self.declare_parameter('goal_tolerance_m', 0.20)
        self.declare_parameter('settle_time_s', 1.0)
        self.declare_parameter('odom_timeout_s', 1.0)
        self.declare_parameter('require_calibration', True)
        self.declare_parameter('fixed_z_m', -3.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.goal = None
        self.odom = None
        self.last_odom_time = None
        self.calibrated = False
        self.fault = None
        self.arrival_since = None
        self.state = 'WAITING_FOR_ODOM'

        self.create_subscription(PoseStamped, '/planner/goal', self.on_goal, 10)
        self.create_subscription(Odometry, '/state/odometry', self.on_odom, 10)
        self.create_subscription(String, '/calibration/status', self.on_calibration, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/controller/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/planner/status', 10)
        self.reached_pub = self.create_publisher(Bool, '/planner/goal_reached', 10)
        self.mode_pub = self.create_publisher(String, '/planner/mode', 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.step)
        self.get_logger().info('Master Planner online; supervision only, no thruster actuation')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_goal(self, msg):
        if msg.header.frame_id not in ('', 'odom'):
            self.get_logger().error('Rejected goal: frame must be odom')
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        if not all(math.isfinite(v) for v in (x, y)):
            self.get_logger().error('Rejected non-finite goal')
            return
        self.goal = (x, y)
        self.arrival_since = None
        self.fault = None
        self.get_logger().info(f'Goal received: x={x:.2f}, y={y:.2f}')

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().warn('Ignoring odometry not in odom frame')
            return
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if not all(math.isfinite(v) for v in xy):
            return
        self.odom = xy
        self.last_odom_time = self.now_s()

    def on_calibration(self, msg):
        try:
            data = json.loads(msg.data)
            # The calibration node currently exposes calibration_complete.
            self.calibrated = data.get('calibration_complete') is True
        except (ValueError, TypeError):
            self.calibrated = False

    def publish_reached(self, value):
        msg = Bool()
        msg.data = value
        self.reached_pub.publish(msg)

    def step(self):
        now = self.now_s()
        timeout = float(self.get_parameter('odom_timeout_s').value)
        if self.last_odom_time is None:
            self.state = 'WAITING_FOR_ODOM'
        elif now - self.last_odom_time > timeout:
            self.state = 'FAULT'
            self.fault = 'ODOMETRY_TIMEOUT'
        elif self.get_parameter('require_calibration').value and not self.calibrated:
            self.state = 'WAITING_FOR_CALIBRATION'
            self.arrival_since = None
        elif self.goal is None:
            self.state = 'READY'
        else:
            dx = self.goal[0] - self.odom[0]
            dy = self.goal[1] - self.odom[1]
            distance = math.hypot(dx, dy)
            if distance <= float(self.get_parameter('goal_tolerance_m').value):
                if self.arrival_since is None:
                    self.arrival_since = now
                if now - self.arrival_since >= float(self.get_parameter('settle_time_s').value):
                    self.state = 'GOAL_REACHED'
                    self.goal = None
                else:
                    self.state = 'NAVIGATING'
            else:
                self.arrival_since = None
                self.state = 'NAVIGATING'

            if self.state == 'NAVIGATING':
                target = PoseStamped()
                target.header.stamp = self.get_clock().now().to_msg()
                target.header.frame_id = 'odom'
                target.pose.position.x = self.goal[0]
                target.pose.position.y = self.goal[1]
                # Placeholder only: depth control is out of scope for V1.
                target.pose.position.z = float(self.get_parameter('fixed_z_m').value)
                target.pose.orientation.w = 1.0
                self.target_pub.publish(target)

        status = {
            'state': self.state,
            'goal_xy': list(self.goal) if self.goal is not None else None,
            'current_xy': list(self.odom) if self.odom is not None else None,
            'calibrated': self.calibrated,
            'fault': self.fault if self.state == 'FAULT' else None,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
        mode_msg = String()
        mode_msg.data = self.state
        self.mode_pub.publish(mode_msg)
        self.publish_reached(self.state == 'GOAL_REACHED')


def main(args=None):
    rclpy.init(args=args)
    node = MasterPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
