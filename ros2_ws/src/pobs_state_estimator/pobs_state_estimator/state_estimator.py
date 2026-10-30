import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped # swap to pointStamp befor push
from nav_msgs.msg import Odometry

def rotate_vector_by_quaternion(v, q):
    """Rotate a 3D vector from body coordinates into world coordinates."""
    x, y, z, w = q
    vx, vy, vz = v
    # Quaternion-derived rotation matrix
    return (
        (1 - 2*(y*y + z*z))*vx
        + 2*(x*y - z*w)*vy
        + 2*(x*z + y*w)*vz,
        2*(x*y + z*w)*vx
        + (1 - 2*(x*x + z*z))*vy
        + 2*(y*z - x*w)*vz,
        2*(x*z - y*w)*vx
        + 2*(y*z + x*w)*vy
        + (1 - 2*(x*x + y*y))*vz,
    )
def rotate_world_to_body(v, q):
    """Rotate a vector from world coordinates into body coordinates."""
    x, y, z, w = q
    q_inverse = (-x, -y, -z, w)
    return rotate_vector_by_quaternion(v, q_inverse)

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')

        self.declare_parameter('imu_topic', '/pobs/imu')

        self.imu_topic = self.get_parameter('imu_topic').value

        self.declare_parameter('depth_topic','/pobs/depth')
        self.declare_parameter('max_depth_dt_s',0.5)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.max_depth_dt_s = float(self.get_parameter('max_depth_dt_s').value)
        self.depth_m = None
        self.vertical_velocity_ms = 0.0
        self.last_depth_stamp = None

        self.declare_parameter('publish_rate_hz',100.0)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.orientation = None
        self.angular_velocity = None
        self.last_imu_stamp = None
        self.world_acceleration = None

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.last_integration_stamp = None

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            50,
        )
        self.get_logger().info(f'State estimator listening to {self.imu_topic}')
        self.depth_sub = self.create_subscription(
            PointStamped,
            self.depth_topic,
            self.depth_callback,
            50,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/state/odometry',
            10,
        )
        self.get_logger().info('Odometry publisher created')
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_odometry,
        )
        self.get_logger().info(
            f'Odometry timer created at {self.publish_rate} Hz'
)

    def imu_callback(self,msg:Imu):
        q = msg.orientation
        norm = math.sqrt(q.x**2 +q.y**2 + q.z**2 + q.w**2)
        if norm < 1e-8 or msg.orientation_covariance[0] ==-1:
            self.get_logger().warning('IMU orientation unavailable  ')
            return
        self.orientation=(q.x/norm, q.y/norm, q.z/norm, q.w/norm )
        accel_body = (msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,)
        accel_world = rotate_vector_by_quaternion(accel_body,self.orientation)
        self.world_acceleration = (
            accel_world[0],
            accel_world[1],
            accel_world[2] - 9.81,
        )
        self.get_logger().info(
            f'Gravity-compensated accel: '
            f'ax={self.world_acceleration[0]:.4f}, '
            f'ay={self.world_acceleration[1]:.4f}, '
            f'az={self.world_acceleration[2]:.4f}',
            throttle_duration_sec=2.0,
)
        self.angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,)
        self.last_imu_stamp = msg.header.stamp
        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )
        if stamp <= 0.0:
            return
        if self.last_integration_stamp is None:
            self.last_integration_stamp = stamp
            return
        dt = stamp - self.last_integration_stamp
        if dt == 0.0: # duplicate imu timstamp
            return
        if dt < 0.0: # sim clock moved backward - reset integration state
            self.last_integration_stamp = stamp
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.position_x=0.0
            self.position_y=0.0
            return

        self.last_integration_stamp = stamp

        #gravity compensated h acc
        ax = self.world_acceleration[0]
        ay = self.world_acceleration[1]
        # Ignore small acceleration fluctuations caused by sensor noise
        accel_deadband = 0.01  # m/s^2
        if abs(ax) < accel_deadband:
            ax = 0.0
        if abs(ay) < accel_deadband:
            ay = 0.0
        #update pos suign current velocity and ac
        self.position_x += self.velocity_x * dt +0.5 * ax * dt**2
        self.position_y += self.velocity_y * dt +0.5 * ay * dt**2
        #update v
        self.velocity_x += ax* dt
        self.velocity_y  += ay*dt

        self.get_logger().info(
            f'IMU received: '
            f'w={self.orientation[3]:.3f}, '
            f'gyro_z={self.angular_velocity[2]:.4f}',
            throttle_duration_sec=2.0,
            )
    def depth_callback(self, msg:PointStamped):
        # Absolute positive water depth, not the downward sonar's bottom clearance.
        depth = float(msg.point.z)
        stamp = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if not math.isfinite(depth) or depth < 0.0:
            return
        if stamp <= 0.0:
            return
        if self.last_depth_stamp is not None:
            dt = stamp - self.last_depth_stamp
            if dt  <= 0.0:
                # backward timetsmpa may indicate a sim reset
                self.last_depth_stamp = None
                self.depth_m = None
                self.vertical_velocity_ms=0.0
                return
            if dt <= self.max_depth_dt_s:
                self.vertical_velocity_ms = ((depth-self.depth_m)/dt)
            else:
                self.vertical_velocity_ms = 0.0
        self.depth_m = depth
        self.last_depth_stamp = stamp
        self.get_logger().info(
            f'Depth: {self.depth_m:.3f} m, '
            f'depth rate: {self.vertical_velocity_ms:.3f} m/s',
            throttle_duration_sec=2.0,
        )
    def publish_odometry(self):
        self.get_logger().info(
        f'IMU ready: {self.orientation is not None}, '
        f'Angular velocity ready: {self.angular_velocity is not None}, '
        f'Depth ready: {self.depth_m is not None}',
        throttle_duration_sec=2.0,
    )
        if(self.orientation is None or self.angular_velocity is None or self.depth_m is None):
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = - self.depth_m
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = self.orientation
        velocity_body = rotate_world_to_body(
            (
                self.velocity_x,
                self.velocity_y,
                -self.vertical_velocity_ms,
            ),
            self.orientation,
        )
        odom.twist.twist.linear.x = velocity_body[0]
        odom.twist.twist.linear.y = velocity_body[1]
        odom.twist.twist.linear.z = velocity_body[2]
        (
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
        ) = self.angular_velocity
        # Pose covariance order: x, y, z, roll, pitch, yaw
        pose_cov = [0.0] * 36

        pose_cov[0] = 100.0     # X position uncertainty (m^2)
        pose_cov[7] = 100.0     # Y position uncertainty (m^2)
        pose_cov[14] = 0.04     # Depth-derived Z uncertainty (m^2)
        pose_cov[21] = 0.01     # Roll uncertainty (rad^2)
        pose_cov[28] = 0.01     # Pitch uncertainty (rad^2)
        pose_cov[35] = 0.04     # Yaw uncertainty (rad^2)

        odom.pose.covariance = pose_cov

        # Twist covariance order: vx, vy, vz, wx, wy, wz
        twist_cov = [0.0] * 36

        twist_cov[0] = 25.0      # X velocity uncertainty
        twist_cov[7] = 25.0      # Y velocity uncertainty
        twist_cov[14] = 0.25     # Vertical velocity uncertainty
        twist_cov[21] = 0.0004   # Angular velocity X
        twist_cov[28] = 0.0004   # Angular velocity Y
        twist_cov[35] = 0.0004   # Angular velocity Z

        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
