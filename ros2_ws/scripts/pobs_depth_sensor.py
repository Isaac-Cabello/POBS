import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
class Depth(Node):
    def __init__(self):
        super().__init__('pobs_depth_sensor')
        self.depth=self.create_publisher(Float64,'/pobs/depth',10)
        self.pressure=self.create_publisher(FluidPressure,'/pobs/pressure',10)
        self.create_subscription(PoseStamped,'/model/pobs/pose',self.update,10)
    def update(self,msg):
        d=max(0.0,-msg.pose.position.z)
        self.depth.publish(Float64(data=d))
        p=FluidPressure();p.header=msg.header;p.header.frame_id='base_link'
        p.fluid_pressure=101325.0+1000.0*9.81*d;p.variance=0.0
        self.pressure.publish(p)
rclpy.init()
n=Depth()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.shutdown()
