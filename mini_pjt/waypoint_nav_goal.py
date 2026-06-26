
# waypoint_nav_goal.py
# 'web_true' 토픽을 받으면 waypoint 3곳을 이동한 뒤 rc_pos 목표를 받아 이동하는 노드.
# ros2 topic pub --once /robot2/web_true std_msgs/msg/Bool "{data: true}"

import math
from tf2_ros import Buffer, TransformListener
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool, String
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
from geometry_msgs.msg import PoseWithCovarianceStamped

INITIAL_POSE_POSITION = [-0.0, 0.0]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH
STOP_DIST = 1 # 70cm
POINT_1 = ([-2.8005542755126953, 0.219985693693161], TurtleBot4Directions.WEST)
POINT_2 = ([-2.76662278175354, 3.821401357650757], TurtleBot4Directions.SOUTH)
# POINT_3 = ([-3.1925582885742188, 3.934382200241089], TurtleBot4Directions.SOUTH)

ROUTE = [
    POINT_1,
    POINT_2,
    # POINT_3,
]


class WaypointNavGoal(Node):
    def __init__(self):
        super().__init__('waypoint_nav_goal')
        self.navigator = TurtleBot4Navigator()
        self.initialize_navigation()

        self.robot_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)

        self.web_sub = self.create_subscription(Bool, 'web_true', self.web_callback, 10)
        self.rc_pos_sub = self.create_subscription(
            PointStamped,
            'rc_pos',
            self.rc_pos_callback,
            10,
        )
        self.dist_sub = self.create_subscription(String, 'depth_distance', self.deist_callback, 10)
        self.timer = self.create_timer(0.2, self.update)
        self.state = 'IDLE'
        self.route_index = 0

        self.get_logger().info('Waypoint nav goal node has been started.')

    def initialize_navigation(self):
        initial_pose = self.navigator.getPoseStamped(
            INITIAL_POSE_POSITION,
            INITIAL_POSE_DIRECTION,
        )
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Initial pose is set and Nav2 is active.')

    def amcl_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        robot_xy = self.robot_pose
        if robot_xy is None:
            return


    def web_callback(self, msg):

        if not msg.data or self.state != 'IDLE':
            return

        if self.navigator.getDockedStatus():
            self.navigator.undock()

        self.route_index = 0
        self.state = 'GO_NEXT_WAYPOINT'
        self.get_logger().info('Web trigger received. Starting waypoint route.')

    def update(self):
        if self.state == 'GO_NEXT_WAYPOINT':
            if self.route_index >= len(ROUTE):
                self.state = 'WAIT_RC_POS'
                self.get_logger().info('Waypoint route completed. Waiting for rc_pos.')
                return

            position, direction = ROUTE[self.route_index]
            pose = self.navigator.getPoseStamped(position, direction)
            self.navigator.goToPose(pose)
            self.get_logger().info(
                f'Moving to waypoint {self.route_index + 1}/{len(ROUTE)}.'
            )
            self.state = 'WAIT_WAYPOINT'

        elif self.state == 'WAIT_WAYPOINT':
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.route_index += 1
                    self.state = 'GO_NEXT_WAYPOINT'
                else:
                    self.get_logger().warn('Failed to reach waypoint.')
                    self.state = 'IDLE'

        elif self.state == 'WAIT_RC_GOAL':
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('rc_pos navigation goal reached.')
                else:
                    self.get_logger().warn('Failed to reach rc_pos navigation goal.')
                self.state = 'WAIT_RC_POS'
    
    
    
    def deist_callback(self, msg):
        self.get_logger().info(f"Received depth distance: {msg.data}")
        self.distance = float(msg.data)
  

    def rc_pos_callback(self, msg):
        if self.state not in ('WAIT_RC_POS', 'WAIT_RC_GOAL'):
            return

        car_x, car_y = msg.point.x, msg.point.y
        
        robot_xy = self.robot_pose
        if robot_xy is None:    
            return
        
        robot_x, robot_y = robot_xy

        dist_to_car = math.hypot(car_x - robot_x, car_y - robot_y)
        self.get_logger().info(f'AMR-RC카 거리: {dist_to_car:.2f}m')

        if dist_to_car <= STOP_DIST:
        # 70cm 이내 도달 → 정지, Nav2 목표 취소
            if self.state == 'WAIT_RC_GOAL':
                self.navigator.cancelTask()
                self.get_logger().info(f'목표 지점({STOP_DIST}m) 도달. 정지.')
                # self.get_logger().info(f'목표 지점({dist_to_car}m) 거리 임.')
            self.state = 'WAIT_RC_POS' # 또는 원하는 상태로
            return

        # 아직 70cm보다 멀면 → 계속 차를 향해 이동
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = car_x
        goal_pose.pose.position.y = car_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.navigator.goToPose(goal_pose)
        # self.last_goal_xy = (car_x, car_y)
        self.state = 'WAIT_RC_GOAL'


def main():
    rclpy.init()
    node = WaypointNavGoal()
    try:
        rclpy.spin(node)
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
