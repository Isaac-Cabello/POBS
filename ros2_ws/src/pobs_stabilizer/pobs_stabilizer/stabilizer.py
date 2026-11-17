
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped, WrenchStamped
from nav_msgs.msg import Odometry

from pobs_stabilizer.model_properties import load_model_properties


# ============================================================
# MODEL PROPERTIES
# ============================================================

def calculate_model_properties():
    gazebo_folder = Path.home() / "ros2_ws/src/pobs_gazebo"

    properties = load_model_properties(
        gazebo_folder / "models/pobs/model.sdf",
        gazebo_folder / "worlds/lake.sdf",
    )

    total_mass = 0.0
    total_volume = 0.0

    mass_sum = [0.0, 0.0, 0.0]
    volume_sum = [0.0, 0.0, 0.0]

    # Assumes the current SDF geometry:
    # unrotated body and zero offsets on other links.
    for link in properties["links"]:
        mass = link["mass"]

        com = [
            link["pose"][i] + link["inertial_pose"][i]
            for i in range(3)
        ]

        total_mass += mass

        for i in range(3):
            mass_sum[i] += mass * com[i]

        for collision in link["collisions"]:
            length, width, height = collision["size"]
            volume = length * width * height

            center = [
                link["pose"][i] + collision["pose"][i]
                for i in range(3)
            ]

            total_volume += volume

            for i in range(3):
                volume_sum[i] += volume * center[i]

    center_of_mass = [value / total_mass for value in mass_sum]
    center_of_buoyancy = [value / total_volume for value in volume_sum]

    g = -properties["gravity"][2]

    weight = total_mass * g
    buoyancy = properties["water_density"] * total_volume * g

    print("Total mass (kg):", total_mass)
    print("Displaced volume (m^3):", total_volume)
    print("Center of mass (m):", center_of_mass)
    print("Center of buoyancy (m):", center_of_buoyancy)

    return weight, buoyancy, center_of_mass, center_of_buoyancy


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


# ============================================================
# STABILIZER NODE
# ============================================================

class Stabilizer(Node):

    def __init__(self):
        super().__init__("pobs_stabilizer")

        # Use Gazebo /clock rather than wall time for timers and PID dt.
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        # ====================================================
        # PID STATE
        # ====================================================

        self.roll_p_error = None
        self.pitch_p_error = None
        self.yaw_p_error = None
        self.depth_p_error = None
        self.x_p_error = None

        # I: Integral accumulators
        self.roll_integral = 0.0
        self.pitch_integral = 0.0000
        self.yaw_integral = 0.0
        self.depth_integral = 0.0
        self.x_integral = 0.0

        # Previous errors for unsmoothed (sample-to-sample) D terms.
        self.old_roll_error = None
        self.old_pitch_error = None
        self.old_yaw_error = None
        self.old_depth_error = None
        self.old_x_error = None

        # Timing
        self.last_update_time = None
        self.dt = 0.0

        # Targets and safety limits. Units: m, s (simulation time), N.
        self.declare_parameter("desired_depth", 1.0)
        self.declare_parameter("depth_force_limit", 30.0)
        self.declare_parameter("depth_timeout", 1.0)
        self.declare_parameter("orientation_timeout", 1.0)

        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = None  # Hold the initial heading once orientation arrives.
        self.desired_x_velocity = 0.0  # Hold zero forward/backward speed.

        self.desired_depth = float(
            self.get_parameter("desired_depth").value
        )
        self.depth_force_limit = float(
            self.get_parameter("depth_force_limit").value
        )
        self.depth_timeout = float(
            self.get_parameter("depth_timeout").value
        )
        self.orientation_timeout = float(
            self.get_parameter("orientation_timeout").value
        )

        if (not math.isfinite(self.desired_depth) or self.desired_depth < 0 or
            not math.isfinite(self.depth_force_limit) or self.depth_force_limit <= 0 or
            not math.isfinite(self.depth_timeout) or self.depth_timeout <= 0 or
            not math.isfinite(self.orientation_timeout) or self.orientation_timeout <= 0):
            raise ValueError(
                "Invalid desired_depth, depth_force_limit or sensor timeout"
            )

        self.pitch_kp = 20.0
        self.pitch_kd = 15.0

        # Initial gains for the newly enabled axes; tune in simulation.
        self.roll_kp = 20.0
        self.roll_kd = 10.0
        self.yaw_kp = 15.0
        self.yaw_kd = 8.0
        # Forward speed error is already a velocity error, so P acts as damping.
        self.x_kp = 20.0
        self.x_kd = 0.0

        # Limits on new axes to avoid runaway thrust during initial tuning.
        self.roll_torque_limit = 10.0
        self.yaw_torque_limit = 10.0
        self.x_force_limit = 20.0

        self.depth_kp = 100.0
        self.depth_kd = 100.0

        self.depth = None
        self.last_depth_time = None
        self.last_orientation_time = None
        self.last_velocity_time = None
        self.forward_velocity = None
        self.depth_is_fresh = False
        self._last_status = None

        # Load physical properties once at startup.
        (
            self.weight,
            self.buoyancy,
            self.center_of_mass,
            self.center_of_buoyancy,
        ) = calculate_model_properties()

        # Latest orientation from the state estimator.
        self.orientation = None

        # Subscribe to state estimator.
        self.state_sub = self.create_subscription(
            Odometry,
            "/state/odometry",
            self.state_callback,
            10
        )

        # Subscribe to depth sensor.
        self.depth_sub = self.create_subscription(
            PointStamped,
            "/pobs/depth",
            self.depth_callback,
            10
        )

        # Publish desired wrench to thruster allocator.
        self.publisher = self.create_publisher(
            WrenchStamped,
            "/cmd_wrench",
            10
        )

        # Run controller at nominally 10 Hz.
        self.timer = self.create_timer(
            0.1,
            self.publish_command
        )

    # ========================================================
    # SENSOR INPUTS AND TIMING
    # ========================================================

    def state_callback(self, msg):
        q = msg.pose.pose.orientation

        norm = math.sqrt(
            q.x**2 + q.y**2 + q.z**2 + q.w**2
        )

        if not math.isfinite(norm) or norm < 1e-8:
            return

        self.orientation = (
            q.x / norm,
            q.y / norm,
            q.z / norm,
            q.w / norm
        )

        self.last_orientation_time = self.sim_seconds()

        # nav_msgs/Odometry.twist is defined in the child frame (body frame).
        # Do not use a velocity expressed in a different named frame.
        vx = msg.twist.twist.linear.x
        if (msg.child_frame_id in ("", "base_link") and math.isfinite(vx)):
            self.forward_velocity = float(vx)
            self.last_velocity_time = self.sim_seconds()
        else:
            self.forward_velocity = None
            self.last_velocity_time = None

    def depth_callback(self, msg):
        if math.isfinite(msg.point.z):
            self.depth = msg.point.z
            self.last_depth_time = self.sim_seconds()

    def sim_seconds(self):
        """ROS clock; with use_sim_time=True this is /clock time."""
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def is_fresh(stamp, now, timeout):
        return (
            stamp is not None
            and 0 <= now - stamp <= timeout
        )

    def report_status(self, status):
        """Only log when sensor state changes."""

        if status != self._last_status:
            if status == "depth_stale":
                self.get_logger().warning(
                    "Depth unavailable/stale: publishing zero wrench"
                )

            elif status == "orientation_stale":
                self.get_logger().warning(
                    "Orientation unavailable/stale: assuming level body; "
                    "pitch control disabled"
                )

            elif status == "normal":
                self.get_logger().info(
                    "Depth and orientation data available"
                )

            self._last_status = status

    def publish_zero_wrench(self):
        """Replace the previous wrench with zero when depth is stale."""

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        self.publisher.publish(msg)

    def calculate_pitch(self):
        qx, qy, qz, qw = self.orientation
        value = 2 * (qw*qy - qz*qx)
        return math.asin(max(-1.0, min(1.0, value)))

    def calculate_roll(self):
        qx, qy, qz, qw = self.orientation
        return math.atan2(
            2 * (qw*qx + qy*qz),
            1 - 2 * (qx*qx + qy*qy),
        )

    def calculate_yaw(self):
        qx, qy, qz, qw = self.orientation
        return math.atan2(
            2 * (qw*qz + qx*qy),
            1 - 2 * (qy*qy + qz*qz),
        )

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ========================================================
    # COORDINATE TRANSFORMATIONS
    # ========================================================

    def calculate_up_body(self, orientation=None):
        """
        World-up unit vector expressed in body coordinates.

        R^T * [0, 0, 1]

        Without fresh orientation, assume a level vehicle.
        This degraded mode is not accurate when tilted.
        """

        if orientation is None:
            return [0.0, 0.0, 1.0]

        qx, qy, qz, qw = orientation

        return [
            2 * (qx*qz - qw*qy),
            2 * (qy*qz + qw*qx),
            1 - 2 * (qx*qx + qy*qy),
        ]

    # ========================================================
    # FEEDFORWARD CONTROL
    # ========================================================

    def calculate_feedforward(self, up_body):
        """Calculate gravity and buoyancy compensation about COM."""

        # Gravity and buoyancy in body coordinates.
        gravity_force = [
            -self.weight * direction
            for direction in up_body
        ]

        buoyancy_force = [
            self.buoyancy * direction
            for direction in up_body
        ]

        # Force required to cancel gravity and buoyancy.
        feedforward_force = [
            -(gravity_force[i] + buoyancy_force[i])
            for i in range(3)
        ]

        # Moment arm from COM to COB.
        buoyancy_arm = [
            self.center_of_buoyancy[i] - self.center_of_mass[i]
            for i in range(3)
        ]

        # Gravity acts at COM, so it produces no torque about COM.
        buoyancy_torque = cross(
            buoyancy_arm,
            buoyancy_force
        )

        # Torque required to cancel buoyancy torque.
        feedforward_torque = [
            -torque
            for torque in buoyancy_torque
        ]

        return feedforward_force, feedforward_torque

    def calculate_vertical_force_body(
        self,
        upward_force_newtons,
        up_body
    ):
        """
        Convert world-vertical force into body-frame force.

        Positive force means upward in world coordinates.

        The allocator receives the equivalent body-frame
        X, Y, and Z force components.
        """

        return [
            upward_force_newtons * component
            for component in up_body
        ]

    # ========================================================
    # PID CALCULATIONS
    # ========================================================

    def calculate_roll_pid(self, roll_error, dt):
        self.roll_p_error = roll_error
        self.roll_integral += roll_error * dt

        roll_derivative = 0.0
        if self.old_roll_error is not None and dt > 0:
            roll_derivative = self.wrap_angle(
                roll_error - self.old_roll_error
            ) / dt
        self.old_roll_error = roll_error

        roll_torque = (
            self.roll_kp * roll_error
            + self.roll_kd * roll_derivative
        )
        return max(-self.roll_torque_limit, min(self.roll_torque_limit, roll_torque))

    def calculate_pitch_pid(self, pitch_error, dt):

        # Raw proportional and derivative errors (no smoothing).
        self.pitch_p_error = pitch_error
        self.pitch_integral += pitch_error * dt

        pitch_derivative = 0.0
        if self.old_pitch_error is not None and dt > 0:
            pitch_derivative = (pitch_error - self.old_pitch_error) / dt

        self.old_pitch_error = pitch_error

        pitch_torque = (
            self.pitch_kp * self.pitch_p_error
            + self.pitch_kd * pitch_derivative
        )
        return pitch_torque

    def calculate_yaw_pid(self, yaw_error, dt):
        # Wrap error differences to avoid a full-turn derivative spike at +/-pi.
        yaw_derivative = 0.0
        if self.old_yaw_error is not None and dt > 0:
            yaw_derivative = self.wrap_angle(yaw_error - self.old_yaw_error) / dt
        self.old_yaw_error = yaw_error
        self.yaw_p_error = yaw_error
        self.yaw_integral += yaw_error * dt

        yaw_torque = (
            self.yaw_kp * yaw_error
            + self.yaw_kd * yaw_derivative
        )
        return max(-self.yaw_torque_limit, min(self.yaw_torque_limit, yaw_torque))

    def calculate_depth_pid(self, depth_error, dt):
        self.depth_p_error = depth_error
        self.depth_integral += depth_error * dt

        depth_derivative = 0.0
        if self.old_depth_error is not None and dt > 0:
            depth_derivative = (depth_error - self.old_depth_error) / dt
        self.old_depth_error = depth_error

        requested_upward_force = (
            self.depth_kp * depth_error
            + self.depth_kd * depth_derivative
        )
        return requested_upward_force

    def calculate_velocity_pid(self, x_error, dt):
        self.x_p_error = x_error
        self.x_integral += x_error * dt

        x_derivative = 0.0
        if self.old_x_error is not None and dt > 0:
            x_derivative = (x_error - self.old_x_error) / dt
        self.old_x_error = x_error

        x_force = (
            self.x_kp * x_error
            + self.x_kd * x_derivative
        )
        return max(-self.x_force_limit, min(self.x_force_limit, x_force))

    def calculate_feedback(
        self,
        roll_error,
        pitch_error,
        yaw_error,
        depth_error,
        x_error,
        dt
    ):

        feedback_force = [
            self.calculate_velocity_pid(x_error, dt),
            0.0,
            self.calculate_depth_pid(depth_error, dt)
        ]

        feedback_torque = [
            self.calculate_roll_pid(roll_error, dt),
            self.calculate_pitch_pid(pitch_error, dt),
            self.calculate_yaw_pid(yaw_error, dt)
        ]

        return feedback_force, feedback_torque

    # ========================================================
    # MAIN CONTROL LOOP
    # ========================================================

    def publish_command(self):

        # ROS clock uses simulation time.
        now = self.sim_seconds()

        # Depth is mandatory.
        # Orientation from the estimator is optional.
        if not self.is_fresh(
            self.last_depth_time,
            now,
            self.depth_timeout
        ):

            self.report_status("depth_stale")

            self.last_update_time = None

            self.depth_p_error = None
            self.old_depth_error = None
            self.pitch_p_error = None
            self.old_pitch_error = None
            self.roll_p_error = None
            self.old_roll_error = None
            self.yaw_p_error = None
            self.old_yaw_error = None
            self.x_p_error = None
            self.old_x_error = None

            self.publish_zero_wrench()
            return

        # Check whether orientation is still valid.
        orientation_fresh = self.is_fresh(
            self.last_orientation_time,
            now,
            self.orientation_timeout
        )

        # Transform world-up into body coordinates.
        # Assume a level body if orientation is unavailable.
        up_body = self.calculate_up_body(
            self.orientation if orientation_fresh else None
        )

        self.report_status(
            "normal" if orientation_fresh else "orientation_stale"
        )

        # Calculate elapsed simulation time.
        if self.last_update_time is None:
            dt = 0.0
        else:
            dt = now - self.last_update_time

            if dt < 0.0:
                self.old_depth_error = None
                self.old_roll_error = None
                self.old_x_error = None
                self.old_pitch_error = None
                self.old_yaw_error = None
                self.desired_yaw = None
                dt = 0.0

            elif dt == 0.0:
                return

        self.last_update_time = now

        # Gravity/buoyancy compensation about COM.
        feedforward_force, feedforward_torque = (
            self.calculate_feedforward(up_body)
        )

        # ====================================================
        # DEPTH CONTROL
        # ====================================================

        # Depth positive down.
        # Positive error => too deep => upward force.
        depth_error = self.depth - self.desired_depth

        requested_upward_force = self.calculate_depth_pid(
            depth_error,
            dt
        )

        # Saturate depth PD correction to configured force limit.
        upward_force = max(
            -self.depth_force_limit,
            min(
                self.depth_force_limit,
                requested_upward_force
            )
        )

        # Convert world-vertical force into body-frame force.
        depth_force_body = self.calculate_vertical_force_body(
            upward_force,
            up_body
        )

        # Add vertical feedback to gravity/buoyancy feedforward.
        desired_force = [
            feedforward_force[i] + depth_force_body[i]
            for i in range(3)
        ]

        # Start with gravity/buoyancy torque compensation.
        desired_torque = list(feedforward_torque)

        # Roll, pitch, and yaw feedback require current orientation.
        if orientation_fresh:
            roll = self.calculate_roll()
            pitch = self.calculate_pitch()
            yaw = self.calculate_yaw()

            if self.desired_yaw is None:
                self.desired_yaw = yaw
                self.get_logger().info(
                    f"Holding initial yaw heading: {math.degrees(yaw):.1f} degrees"
                )

            roll_error = self.wrap_angle(self.desired_roll - roll)
            pitch_error = self.desired_pitch - pitch
            yaw_error = self.wrap_angle(self.desired_yaw - yaw)

            desired_torque[0] += self.calculate_roll_pid(roll_error, dt)
            desired_torque[1] += self.calculate_pitch_pid(pitch_error, dt)
            desired_torque[2] += self.calculate_yaw_pid(yaw_error, dt)

            # Hold zero forward velocity, if Odometry supplies body-X speed.
            if (self.forward_velocity is not None and self.is_fresh(
                self.last_velocity_time, now, self.orientation_timeout
            )):
                x_error = self.desired_x_velocity - self.forward_velocity
                desired_force[0] += self.calculate_velocity_pid(x_error, dt)
            else:
                self.x_p_error = None
                self.old_x_error = None
        else:
            self.pitch_p_error = None
            self.old_pitch_error = None
            self.roll_p_error = None
            self.old_roll_error = None
            self.yaw_p_error = None
            self.old_yaw_error = None
            self.x_p_error = None
            self.old_x_error = None

        # ====================================================
        # PUBLISH WRENCH
        # ========================================================

        msg = WrenchStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.wrench.force.x = desired_force[0]
        msg.wrench.force.y = desired_force[1]
        msg.wrench.force.z = desired_force[2]

        msg.wrench.torque.x = desired_torque[0]
        msg.wrench.torque.y = desired_torque[1]
        msg.wrench.torque.z = desired_torque[2]

        self.publisher.publish(msg)


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    node = Stabilizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
