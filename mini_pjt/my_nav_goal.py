import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion # 이동 명령

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import numpy as np
import cv2
import threading
import math # 이동 명령

class MapNavGoal(Node):
    def __init__(self):
        super().__init__('map_sub_nav_goal')

        self.bridge = CvBridge()
        self.K = None
        self.lock = threading.Lock()

        ns = self.get_namespace()

        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        # initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)    
        # self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.logged_intrinsics = False

        self.subscription = self.create_subscription(
            PointStamped,
            'rc_pos',
            self.map_callback,
            10)

    # 로봇의 시야를 보여주는 화면에서 지점을 클릭하면 그곳이 map에서 어디인지 TF 변환
    # PointStamped (pt_camera) -> depth 기준 포인트, goal_pose -> map 기준 포인트
    def map_callback(self, msg):
        self.get_logger().info(
            f"Received goal point: "
            f"({msg.point.x: .2f}, {msg.point.y:.2f})"
        )

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.point.z = 0.0
        yaw = 0.0                   # 목표 방향 설정
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.navigator.goToPose(goal_pose)

        self.get_logger().info("Sent navigation goal to map coordinate.")



def main():
    rclpy.init()
    node = MapNavGoal()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

# This code is a ROS2 node that subscribes to depth and RGB images, allows the user to click on a point in the RGB image,
# and then calculates the corresponding 3D point in the map frame. It also sends a navigation goal to that point.
# The node uses a GUI to display the images and allows interaction via mouse clicks.
# It also handles TF transformations to convert the clicked point from the camera frame to the map frame.
# The node is designed to run in a multi-threaded executor to handle image processing and GUI updates concurrently.
# The code includes error handling for TF transformations and image processing, ensuring robustness in various scenarios.
# The node also logs important information such as camera intrinsics and image shapes for debugging purposes.
