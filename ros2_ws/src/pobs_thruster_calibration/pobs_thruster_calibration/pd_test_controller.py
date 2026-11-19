import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class PDTestController(Node):
    def __init__(self, mode, duration):
        super().__init__("pd_test_controller")

        self.mode = mode
        self.duration = duration
        self.kp = 0.01
        self.kd = 0.005 if mode == "pd" else 0.0
        self.max_thrust = 0.10
        self.max_tilt = 15.0
        self.imu_timeout = 0.5

        self.latest_imu = None
        self.last_imu_time = None
        self.started = None
        self.finished = False

        self.thruster_publishers = [
            self.create_publisher(
                Float64,
                f"/model/pobs/joint/thruster_{i}_joint/cmd_thrust",
                10,
            )
            for i in range(4)
        ]

        self.create_subscription(
            Imu, "/pobs/imu", self.imu_callback, 50
        )

        self.file = open(f"/tmp/pobs_{mode}_test.csv", "w")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time_s", "pitch_deg", "roll_deg",
            "pitch_rate_dps", "roll_rate_dps",
            "thrust_0", "thrust_1", "thrust_2", "thrust_3"
        ])

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Mode={mode}, Kp={self.kp}, Kd={self.kd}. "
            "Waiting for IMU."
        )

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.last_imu_time = time.monotonic()

    def publish_thrust(self, values):
        for publisher, value in zip(self.thruster_publishers, values):
            msg = Float64()
            msg.data = float(value)
            publisher.publish(msg)

    def stop(self, reason):
        if self.finished:
            return
        self.publish_thrust([0.0] * 4)
        self.finished = True
        self.get_logger().info(reason + " — zero thrust sent")

    def control_loop(self):
        if self.finished:
            return

        now = time.monotonic()

        if self.latest_imu is None:
            return

        if now - self.last_imu_time > self.imu_timeout:
            self.stop("IMU timeout")
            return

        q = self.latest_imu.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

        roll = math.degrees(math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        ))

        pitch_rate = math.degrees(
            self.latest_imu.angular_velocity.y
        )
        roll_rate = math.degrees(
            self.latest_imu.angular_velocity.x
        )

        if abs(pitch) > self.max_tilt or abs(roll) > self.max_tilt:
            self.stop("Tilt safety cutoff")
            return

        if self.started is None:
            self.started = now

        elapsed = now - self.started

        if elapsed >= self.duration:
            self.stop("Test completed")
            return

        pitch_adjust = self.kp * pitch + self.kd * pitch_rate
        roll_adjust = -self.kp * roll - self.kd * roll_rate

        values = [
            pitch_adjust + roll_adjust,
            pitch_adjust - roll_adjust,
            -pitch_adjust + roll_adjust,
            -pitch_adjust - roll_adjust,
        ]

        values = [
            max(-self.max_thrust, min(self.max_thrust, x))
            for x in values
        ]

        self.publish_thrust(values)

        self.writer.writerow([
            round(elapsed, 4),
            pitch, roll, pitch_rate, roll_rate, *values
        ])

    def shutdown(self):
        self.timer.cancel()

        for _ in range(10):
            self.publish_thrust([0.0] * 4)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.file.flush()
        self.file.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["p", "pd"], required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = PDTestController(args.mode, args.duration)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
