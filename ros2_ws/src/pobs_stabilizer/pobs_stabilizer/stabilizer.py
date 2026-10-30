
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from pathlib import Path
from pobs_stabilizer.model_properties import load_model_properties


# Read the model and world files from the container's workspace.
gazebo_folder = Path.home() / "ros2_ws/src/pobs_gazebo"

properties = load_model_properties(
    gazebo_folder / "models/pobs/model.sdf",
    gazebo_folder / "worlds/lake.sdf",
)

total_mass = 0.0
total_volume = 0.0

mass_sum = [0.0, 0.0, 0.0]
volume_sum = [0.0, 0.0, 0.0]

# This matches the current SDF:
# the body is unrotated, and all other inertial offsets are zero.
# The only collision is the box on the unrotated body.
for link in properties["links"]:
    mass = link["mass"]

    # Center of mass of this part, relative to the model origin.
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

        # Center of this collision box, relative to the model origin.
        center = [
            link["pose"][i] + collision["pose"][i]
            for i in range(3)
        ]

        total_volume += volume

        for i in range(3):
            volume_sum[i] += volume * center[i]

center_of_mass = [value / total_mass for value in mass_sum]
center_of_buoyancy = [value / total_volume for value in volume_sum]

print("Total mass (kg):", total_mass)
print("Displaced volume (m^3):", total_volume)
print("Center of mass (m):", center_of_mass)
print("Center of buoyancy (m):", center_of_buoyancy)

# Gravity and buoyancy magnitudes, in newtons.
g = -properties["gravity"][2]

weight = total_mass * g
buoyancy = properties["water_density"] * total_volume * g


def cross(a, b):
    # Cross product of two 3D vectors.
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


class Stabilizer(Node):
    def __init__(self):
        super().__init__("pobs_stabilizer")

        # Latest orientation received from the state estimator.
        self.orientation = None

        # Subscribe to the state estimator's odometry output.
        self.state_sub = self.create_subscription(
            Odometry,
            '/state/odometry',
            self.state_callback,
            10
        )

        # Publish the desired force and torque.
        self.publisher = self.create_publisher(
            Wrench, "/desired_wrench", 10
        )
        self.command_publisher = self.create_publisher(
            WrenchStamped, "/cmd_wrench", 10
        )

        self.timer = self.create_timer(0.1, self.publish_command)

    def state_callback(self, msg):
        # Read the robot's orientation quaternion.
        q = msg.pose.pose.orientation

        # Normalize the quaternion.
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)

        if not math.isfinite(norm) or norm < 1e-8:
            return

        self.orientation = (
            q.x / norm,
            q.y / norm,
            q.z / norm,
            q.w / norm
        )

    def publish_command(self):
        # Wait until the state estimator provides orientation.
        if self.orientation is None:
            return

        qx, qy, qz, qw = self.orientation

        # Calculate world-up direction in robot coordinates.
        up_body = [
            2 * (qx*qz - qw*qy),
            2 * (qy*qz + qw*qx),
            1 - 2 * (qx*qx + qy*qy),
        ]

        gravity_force = [-weight * direction for direction in up_body]
        buoyancy_force = [buoyancy * direction for direction in up_body]

        feedforward_force = [
            -(gravity_force[i] + buoyancy_force[i])
            for i in range(3)
        ]

        gravity_torque = cross(center_of_mass, gravity_force)
        buoyancy_torque = cross(center_of_buoyancy, buoyancy_force)

        feedforward_torque = [
            -(gravity_torque[i] + buoyancy_torque[i])
            for i in range(3)
        ]

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = feedforward_force
        msg.torque.x, msg.torque.y, msg.torque.z = feedforward_torque

        self.publisher.publish(msg)
        command = WrenchStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.wrench = msg
        self.command_publisher.publish(command)


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
