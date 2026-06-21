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

        self.pub = self.create_publisher(PointStamped,'rc_pos', 10)

        self.sub = self.create_subscription(PointStamped, 'center_true', self.rc_callback, 10)

    def rc_callback(self, msg):
        try:
              tf = self.tf_buffer.lookup_transform('map',msg.header.frame_id, rclpy.time.Time())
              pt_map = do_transform_point(msg, tf)
              self.pub.publish(pt_map)

              self.get_logger().info(f"map: x={pt_map.point.x:.2f}, y={pt_map.point.y:.2f}, z={pt_map.point.z:.2f}")
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
