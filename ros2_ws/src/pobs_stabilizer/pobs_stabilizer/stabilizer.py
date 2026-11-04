
import math
from collections import deque
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

        # P: Exponential moving average
        # 80% previous EMA, 20% newest error.
        self.ema_old_weight = 0.8

        self.roll_p_error = None
        self.pitch_p_error = None
        self.yaw_p_error = None
        self.depth_p_error = None
        self.x_p_error = None

        # I: Integral accumulators
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.yaw_integral = 0.0
        self.depth_integral = 0.0
        self.x_integral = 0.0

        # D: Error histories
        self.roll_d_errors = deque(maxlen=10)
        self.pitch_d_errors = deque(maxlen=10)
        self.yaw_d_errors = deque(maxlen=10)
        self.depth_d_errors = deque(maxlen=10)
        self.x_d_errors = deque(maxlen=10)

        # Timing
        self.last_update_time = None
        self.dt = 0.0

        # Pitch and depth PD test settings
        self.desired_pitch = 0.0
        self.desired_depth = 0.5

        self.pitch_kp = 20.0
        self.pitch_kd = 8.0

        self.depth_kp = 100.0
        self.depth_kd = 50.0

        self.depth = None

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
        self.timer = self.create_timer(0.1, self.publish_command)

    # ========================================================
    # STATE ESTIMATION INPUT
    # ========================================================

    def state_callback(self, msg):
        q = msg.pose.pose.orientation

        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)

        if not math.isfinite(norm) or norm < 1e-8:
            return

        self.orientation = (
            q.x / norm,
            q.y / norm,
            q.z / norm,
            q.w / norm
        )

    def depth_callback(self, msg):
        if math.isfinite(msg.point.z):
            self.depth = msg.point.z

    def calculate_pitch(self):
        qx, qy, qz, qw = self.orientation
        value = 2 * (qw*qy - qz*qx)
        return math.asin(max(-1.0, min(1.0, value)))

    # ========================================================
    # COORDINATE TRANSFORMATIONS
    # ========================================================

    def calculate_up_body(self):
        """Express world-up direction in robot coordinates."""

        qx, qy, qz, qw = self.orientation

        return [
            2 * (qx*qz - qw*qy),
            2 * (qy*qz + qw*qx),
            1 - 2 * (qx*qx + qy*qy),
        ]

    # ========================================================
    # FEEDFORWARD CONTROL
    # ========================================================

    def calculate_feedforward(self):
        """Calculate gravity and buoyancy compensation about COM."""

        up_body = self.calculate_up_body()

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
        buoyancy_torque = cross(buoyancy_arm, buoyancy_force)

        # Torque required to cancel buoyancy torque.
        feedforward_torque = [
            -torque
            for torque in buoyancy_torque
        ]

        return feedforward_force, feedforward_torque

    # ========================================================
    # PID CALCULATIONS
    # ========================================================

    def calculate_roll_pid(self, roll_error, dt):

        # P: Exponential moving average
        if self.roll_p_error is None:
            self.roll_p_error = roll_error
        else:
            self.roll_p_error = (
                self.ema_old_weight * self.roll_p_error
                + (1 - self.ema_old_weight) * roll_error
            )

        # I: Accumulate error over time
        self.roll_integral += roll_error * dt

        # D: Compare newest 5 errors to previous 5
        self.roll_d_errors.append(roll_error)

        roll_derivative = 0.0

        if len(self.roll_d_errors) == 10 and dt > 0:
            errors = list(self.roll_d_errors)
            old_avg = sum(errors[:5]) / 5
            new_avg = sum(errors[5:]) / 5

            roll_derivative = (new_avg - old_avg) / (5 * dt)

        # PID gains and output calculation to be added.
        roll_torque = 0.0
        return roll_torque

    def calculate_pitch_pid(self, pitch_error, dt):

        # P: Exponential moving average
        if self.pitch_p_error is None:
            self.pitch_p_error = pitch_error
        else:
            self.pitch_p_error = (
                self.ema_old_weight * self.pitch_p_error
                + (1 - self.ema_old_weight) * pitch_error
            )

        # I: Accumulate error over time
        self.pitch_integral += pitch_error * dt

        # D: Compare newest 5 errors to previous 5
        self.pitch_d_errors.append(pitch_error)

        pitch_derivative = 0.0

        if len(self.pitch_d_errors) == 10 and dt > 0:
            errors = list(self.pitch_d_errors)
            old_avg = sum(errors[:5]) / 5
            new_avg = sum(errors[5:]) / 5

            pitch_derivative = (new_avg - old_avg) / (5 * dt)

        # Pitch PD correction
        pitch_torque = (
            self.pitch_kp * self.pitch_p_error
            + self.pitch_kd * pitch_derivative
        )
        return pitch_torque

    def calculate_yaw_pid(self, yaw_error, dt):

        # P: Exponential moving average
        if self.yaw_p_error is None:
            self.yaw_p_error = yaw_error
        else:
            self.yaw_p_error = (
                self.ema_old_weight * self.yaw_p_error
                + (1 - self.ema_old_weight) * yaw_error
            )

        # I: Accumulate error over time
        self.yaw_integral += yaw_error * dt

        # D: Compare newest 5 errors to previous 5
        self.yaw_d_errors.append(yaw_error)

        yaw_derivative = 0.0

        if len(self.yaw_d_errors) == 10 and dt > 0:
            errors = list(self.yaw_d_errors)
            old_avg = sum(errors[:5]) / 5
            new_avg = sum(errors[5:]) / 5

            yaw_derivative = (new_avg - old_avg) / (5 * dt)

        # PID gains and output calculation to be added.
        yaw_torque = 0.0
        return yaw_torque

    def calculate_depth_pid(self, depth_error, dt):

        # P: Exponential moving average
        if self.depth_p_error is None:
            self.depth_p_error = depth_error
        else:
            self.depth_p_error = (
                self.ema_old_weight * self.depth_p_error
                + (1 - self.ema_old_weight) * depth_error
            )

        # I: Accumulate error over time
        self.depth_integral += depth_error * dt

        # D: Compare newest 5 errors to previous 5
        self.depth_d_errors.append(depth_error)

        depth_derivative = 0.0

        if len(self.depth_d_errors) == 10 and dt > 0:
            errors = list(self.depth_d_errors)
            old_avg = sum(errors[:5]) / 5
            new_avg = sum(errors[5:]) / 5

            depth_derivative = (new_avg - old_avg) / (5 * dt)

        # Depth PD correction
        z_force = (
            self.depth_kp * self.depth_p_error
            + self.depth_kd * depth_derivative
        )
        return z_force

    def calculate_velocity_pid(self, x_error, dt):

        # P: Exponential moving average
        if self.x_p_error is None:
            self.x_p_error = x_error
        else:
            self.x_p_error = (
                self.ema_old_weight * self.x_p_error
                + (1 - self.ema_old_weight) * x_error
            )

        # I: Accumulate error over time
        self.x_integral += x_error * dt

        # D: Compare newest 5 errors to previous 5
        self.x_d_errors.append(x_error)

        x_derivative = 0.0

        if len(self.x_d_errors) == 10 and dt > 0:
            errors = list(self.x_d_errors)
            old_avg = sum(errors[:5]) / 5
            new_avg = sum(errors[5:]) / 5

            x_derivative = (new_avg - old_avg) / (5 * dt)

        # PID gains and output calculation to be added.
        x_force = 0.0
        return x_force

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
            0.0,  # No lateral thrusters
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
        # Wait for orientation and depth data.
        if self.orientation is None or self.depth is None:
            return

        # Measure elapsed SIMULATION time (seconds), not wall-clock time.
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.last_update_time is None:
            self.dt = 0.0
        else:
            self.dt = current_time - self.last_update_time

            if self.dt <= 0.0:
                # Paused, repeated time, or a simulation restart.
                if self.dt < 0.0:
                    self.pitch_d_errors.clear()
                    self.depth_d_errors.clear()
                self.last_update_time = current_time
                return

        self.last_update_time = current_time

        # Calculate feedforward compensation.
        feedforward_force, feedforward_torque = (
            self.calculate_feedforward()
        )

        # Calculate pitch and depth errors.
        pitch_error = self.desired_pitch - self.calculate_pitch()

        # Depth is positive downward.
        # For near-level operation, positive body Z is upward.
        depth_error = self.depth - self.desired_depth

        # Calculate pitch and depth PD corrections.
        pitch_torque = self.calculate_pitch_pid(
            pitch_error, self.dt
        )
        z_force = self.calculate_depth_pid(
            depth_error, self.dt
        )

        # Add feedback corrections to feedforward.
        desired_force = [
            feedforward_force[0],
            feedforward_force[1],
            feedforward_force[2] + z_force
        ]

        desired_torque = [
            feedforward_torque[0],
            feedforward_torque[1] + pitch_torque,
            feedforward_torque[2]
        ]

        # Publish to thruster allocator.
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
