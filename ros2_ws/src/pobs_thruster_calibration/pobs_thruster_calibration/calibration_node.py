import json
import math
import rclpy
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class TCNode(Node):
    def __init__(self):
        super().__init__('thruster_calibration')

        self.declare_parameter('imu_topic','/state/imu')
        self.declare_parameter('max_tilt_deg',5.0)
        self.declare_parameter('imu_timeout_s',1.0)

        imu_topic =self.get_parameter('imu_topic').value
        self.max_tilt = float(self.get_parameter('max_tilt_deg').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout_s').value)

        self.pitch = 0.0
        self.roll =0.0
        self.last_imu_time = None
        self.valid_imu = False
        self.roll_rate = 0.0
        self.pitch_rate =0.0
        self.imu_sample_count = 0
        #calibration  measurement window
        self.declare_parameter('measurement_window_s',5.0)
        self.measurement_window_s = float(self.get_parameter('measurement_window_s').value)
        #store timestmap, pitch, roll for recent IMU reading
        self.imu_measurements =deque()

        self.imu_sub = self.create_subscription(Imu, imu_topic,self.imu_callback,10)
        self.status_pub = self.create_publisher(String, '/calibration/status',10)

        #phase 2
        self.correction_pub = self.create_publisher(String,'/calibration/corrections',10)
        self.measurement_pub = self.create_publisher(String, '/calibration/measurements',10)

        self.declare_parameter('correction_gain',0.01)
        self.declare_parameter('derivative_gain',0.0)
        self.declare_parameter('max_correction',0.10)
        self.correction_gain = float(self.get_parameter('correction_gain').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.derivative_gain = float(self.get_parameter('derivative_gain').value)

        self.declare_parameter('abort_tilt_deg',15.0)
        self.abort_tilt = float(self.get_parameter('abort_tilt_deg').value)
        self.create_timer(0.5,self.publish_status)
        self.get_logger().info('POBS thruster callibraton monitor started ')
        self.get_logger().info('Monitoring IMU only - thrusterd disbaled')


    def imu_callback(self,msg):
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            self.valid_imu = False
            return
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm

        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)

        self.roll =math.degrees(math.atan2(sinr,cosr))
        sinp = 2.0 * (w*y - z*x)
        sinp = max(-1.0, min(1.0,sinp))

        self.pitch = math.degrees(math.asin(sinp))

        #imu v_a is rad
        #these are body-frame rates, nto eular
        self.roll_rate = math.degrees(msg.angular_velocity.x)
        self.pitch_rate = math.degrees(msg.angular_velocity.y)
        self.imu_sample_count +=1
        self.valid_imu = True
        self.last_imu_time = self.get_clock().now()

        #add lastets imu to cal window
        curr_time = self.last_imu_time.nanoseconds * 1e-9
        self.imu_measurements.append((curr_time,self.pitch,self.roll))
        #remove old ones
        cutoff_time = curr_time - self.measurement_window_s
        while(self.imu_measurements and self.imu_measurements[0][0] <cutoff_time):
            self.imu_measurements.popleft()

        sensor_timestamp_ns = (
                msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec
            )
        measurement = {
        'timestamp_ns': sensor_timestamp_ns,
        'pitch_deg': round(self.pitch, 4),
        'roll_deg': round(self.roll, 4),
        'pitch_rate_dps': round(self.pitch_rate, 4),
        'roll_rate_dps': round(self.roll_rate, 4),
        'imu_sample_count': self.imu_sample_count
        }
        msg_out = String()
        msg_out.data = json.dumps(measurement)
        self.measurement_pub.publish(msg_out)
    def calculate_avg_tilt(self):
        if not self.imu_measurements:
            return None, None
        pitch_sum = sum(measurement[1] for measurement in self.imu_measurements)
        roll_sum = sum(measurement[2] for measurement in self.imu_measurements)
        sample_count = len(self.imu_measurements)
        avg_pitch = pitch_sum /sample_count
        avg_roll = roll_sum/ sample_count
        return avg_pitch,avg_roll
    def calculate_corrections(self):
        """
        Calculate diagnostic four-thruster correction patterns.
        These are not verified physical thrust directions.
        No commands are sent to motors.
        """
        pitch_err = self.pitch
        roll_err = -self.roll
        pitch_adjust = (self.correction_gain * pitch_err + self.derivative_gain*self.pitch_rate)
        roll_adjust = self.correction_gain * roll_err + self.derivative_gain*self.roll_rate

        # Hypothetical mixing pattern:
        # 0 = front-left
        # 1 = front-right
        # 2 = rear-left
        # 3 = rear-right

        corrections = [
            pitch_adjust + roll_adjust,
            pitch_adjust - roll_adjust,
            -pitch_adjust +roll_adjust,
            -pitch_adjust - roll_adjust
        ]
        # clamp each candidate correction
        corrections = [max(-self.max_correction,min(self.max_correction,val)) for val in corrections]
        return corrections
    def publish_status(self):
        fresh = False
        if self.valid_imu and self.last_imu_time:
            elapsed = (self.get_clock().now() - self.last_imu_time).nanoseconds /1e9
            fresh = 0 <= elapsed <= self.imu_timeout
        if not fresh:
            self.imu_measurements.clear()
        level = (fresh and abs(self.pitch)<= self.max_tilt and abs(self.roll)<=self.max_tilt)
        unsafe_tilt = (
            fresh and (
                abs(self.pitch) > self.abort_tilt
                or abs(self.roll) > self.abort_tilt
            )
        )
        if fresh:
            avg_pitch, avg_roll = self.calculate_avg_tilt()
        else:
            avg_pitch, avg_roll = None, None
        status =  {
            'state': (
                    'WAITING_FOR_IMU' if not fresh
                    else 'UNSAFE_TILT' if unsafe_tilt
                    else 'MONITORING'
                ),
            'pitch_deg': round(self.pitch, 2) if fresh else None,
            'roll_deg': round(self.roll, 2) if fresh else None,
            'level': level,
            'calibration_complete': False,
            'thrusters_enabled': False,
            'roll_rate_dps': round(self.roll_rate, 3) if fresh else None,
            'pitch_rate_dps': round(self.pitch_rate, 3) if fresh else None,
            'imu_sample_count': self.imu_sample_count,
            'average_pitch_deg': round(avg_pitch, 3) if avg_pitch is not None else None,
            'average_roll_deg': round(avg_roll, 3) if avg_roll is not None else None,
            'window_sample_count': len(self.imu_measurements) if fresh else 0,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

        if fresh:
            if unsafe_tilt:
                corrections = [0.0,0.0,0.0,0.0]
            else:
                corrections = self.calculate_corrections()
            correction_msg = String()
            correction_msg.data = json.dumps({
                'pitch_deg': round(self.pitch,2),
                'roll_deg': round(self.roll,2),
                'candidate_offsets': [round (value,4)
                for value in corrections],
                'mapping_verified':False,
                'thrusters_enabled':False
                })
            self.correction_pub.publish(correction_msg)
def main(args=None):
        rclpy.init(args=args)
        node = TCNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()
