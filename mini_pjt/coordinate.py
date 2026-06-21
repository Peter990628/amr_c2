import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray 
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from rclpy.time import Time 

class Coordinate(Node):
	
	def __init__(self):
		super().__init__('coordinate')
		self.K = None
		self.cx_box = None
		self.cy_box = None
		self.bridge = CvBridge()
		self.yolo_recv_time = None
		
		self.pub_center = self.create_publisher(PointStamped, 'center_true', 10)
		self.camera_info_sub = self.create_subscription(CameraInfo, 'oakd/stereo/camera_info', self.camera_info_callback, 10)
		self.depth_sub = self.create_subscription(Image, 'oakd/stereo/image_raw', self.depth_callback, 10)
		self.yolo_sub = self.create_subscription(Float32MultiArray, 'yolo_pos_amr', self.yolo_callback, 10)
		
	def camera_info_callback(self, msg):
		if self.K is not None:
			return
		self.K = np.array(msg.k).reshape(3, 3)
		self.fx = self.K[0, 0]
		self.fy = self.K[1, 1]
		self.cx_cam = self.K[0, 2]
		self.cy_cam = self.K[1, 2]
		
	
	def yolo_callback(self, msg):
		self.cx_box = int(msg.data[0])
		self.cy_box = int(msg.data[1])
		self.yolo_recv_time = self.get_clock().now()
		self.get_logger().info(f"{self.cx_box}, {self.cy_box}")
		
		# [오후 2:11]pos_msg.data = [float(x1), float(y1), float(x2), float(y2), cx, cy]
		
	def depth_callback(self, msg):
		if self.K is None:
			return
		if self.cx_box is None or self.cy_box is None:
			return
		# if self.yolo_recv_time is None:
		# 	return 
		
		# depth_time = Time.from_msg(msg.header.stamp)
		# time_diff = abs((depth_time - self.yolo_recv_time).nanoseconds / 1e9)
		# if time_diff > 0.15:
		# 	self.get_logger().debug("느리다")
		# 	return
		
		depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		
		h, w = depth_img.shape
		
		if not (0 <= self.cx_box < w and 0 <= self.cy_box < h): 
			self.get_logger().warn(f"out no depth {self.cx_box}, {self.cy_box}")
			return 
		 
		depth = depth_img[self.cy_box, self.cx_box]
		
		if depth <= 0 :
			return
		
		self.depth = depth/1000.0
		self.get_logger().info(f"raw depth = {depth}") #단위 확인용 
		
		X = (self.cx_box - self.cx_cam) * self.depth / self.fx
		Y = (self.cy_box - self.cy_cam) * self.depth / self.fy
		Z = self.depth 
		
		self.get_logger().info(f"Camera Frame: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}") #코드 결과 확인용 
		
		msg_out = PointStamped()
		msg_out.header.frame_id = "oakd_rgb_camera_optical_frame"
		msg_out.header.stamp = msg.header.stamp

		msg_out.point.x = X
		msg_out.point.y = Y
		msg_out.point.z = Z
		self.pub_center.publish(msg_out)
		
def main():
	rclpy.init()
	node = Coordinate()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
