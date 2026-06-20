import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

class Camtomap(Node):

    def __init__(self):
        super().__init__('cam_to_map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped,'/rc_pos', 10)

        self.sub = self.create_subscription(PointStamped, 'center_true', self.rc_callback, 10)

    def rc_callback(self, msg):
        try:
              pt_map = self.tf_buffer.transform(msg, 'map', timeout=Duration(seconds=1.0))
              self.pub.publish(pt_map)
        except Exception as e:
              self.get_logger().warn(f"{e}")

    
def main():
	rclpy.init()
	node = Camtomap()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
