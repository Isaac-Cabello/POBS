"""Simulation-tested XY heading and distance controller.

Publishes normalized DRY-RUN horizontal thruster commands only. It does not
produce physical newton commands or send commands directly to hardware.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class XYController(Node):
    def __init__(self):
        super().__init__('xy_controller')

        self.declare_parameter('require_level', True)
        self.require_level = self.get_parameter('require_level').value

        self.goal = None
        self.pose = None
        self.last_odom = 0.0
        self.last_status = 0.0
        self.planner_state = ''

        self.goal_tolerance = 0.20
        self.max_tilt_deg = 15.0
        self.kp_distance = 0.4
        self.kp_heading = 0.8
        self.max_forward = 0.35
        self.max_turn = 0.30

        self.create_subscription(
            PoseStamped, '/controller/target_pose', self.goal_callback, 10
        )
        self.create_subscription(
            Odometry, '/state/odometry', self.odom_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/planner/status', self.status_callback, 10
        )
        self.left_pub = self.create_publisher(
            Float64, '/planner/dry_run/thruster_4', 10
        )
        self.right_pub = self.create_publisher(
            Float64, '/planner/dry_run/thruster_5', 10
        )
        self.status_pub = self.create_publisher(
            String, '/planner/xy_controller/status', 10
        )
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('XY Controller started - DRY RUN ONLY')

    def goal_callback(self, msg):
        if msg.header.frame_id != 'odom':
            self.goal = None
            self.get_logger().warning('Rejected goal: expected odom frame')
            return
        x = msg.pose.position.x
        y = msg.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self.goal = (x, y)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (
            p.x, p.y, q.x, q.y, q.z, q.w
        )):
            return

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)

        sinp = 2.0 * (q.w*q.y - q.z*q.x)
        pitch = math.asin(clamp(sinp, -1.0, 1.0))

        sinr = 2.0 * (q.w*q.x + q.y*q.z)
        cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr, cosr)

        self.pose = (p.x, p.y, yaw, pitch, roll)
        self.last_odom = time.monotonic()

    def status_callback(self, msg):
        self.last_status = time.monotonic()
        try:
            self.planner_state = json.loads(msg.data).get('state', '')
        except (ValueError, TypeError, AttributeError):
            self.planner_state = ''

    def control_loop(self):
        now = time.monotonic()
        left = 0.0
        right = 0.0
        distance = None
        heading_error = None

        if self.pose is None or now - self.last_odom > 1.0:
            state = 'WAITING_FOR_ODOM'
        elif now - self.last_status > 1.0:
            state = 'WAITING_FOR_PLANNER'
        elif self.planner_state != 'NAVIGATING':
            state = 'INACTIVE'
        elif self.goal is None:
            state = 'WAITING_FOR_GOAL'
        else:
            x, y, yaw, pitch, roll = self.pose
            if self.require_level and (
                abs(math.degrees(pitch)) > self.max_tilt_deg
                or abs(math.degrees(roll)) > self.max_tilt_deg
            ):
                state = 'UNSAFE_TILT'
            else:
                dx = self.goal[0] - x
                dy = self.goal[1] - y
                distance = math.hypot(dx, dy)
                target_heading = math.atan2(dy, dx)
                heading_error = wrap_angle(target_heading - yaw)

                if distance <= self.goal_tolerance:
                    state = 'GOAL_REACHED'
                else:
                    state = 'TRACKING'
                    turn = clamp(
                        self.kp_heading * heading_error,
                        -self.max_turn, self.max_turn
                    )
                    if abs(heading_error) > 0.35:
                        forward = 0.0
                    else:
                        forward = clamp(
                            self.kp_distance * distance,
                            0.0, self.max_forward
                        )
                    # Thruster 4: left (+Y); Thruster 5: right (-Y).
                    left = clamp(forward - turn, -1.0, 1.0)
                    right = clamp(forward + turn, -1.0, 1.0)

        self.left_pub.publish(Float64(data=float(left)))
        self.right_pub.publish(Float64(data=float(right)))
        feedback = {
            'state': state,
            'distance_m': distance,
            'heading_error_deg': (
                math.degrees(heading_error)
                if heading_error is not None else None
            ),
            'thruster_4': left,
            'thruster_5': right,
        }
        self.status_pub.publish(String(data=json.dumps(feedback)))


def main(args=None):
    rclpy.init(args=args)
    node = XYController()
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
