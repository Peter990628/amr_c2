
# waypoint_nav_goal.py
# 'web_true' 토픽을 받으면 waypoint 3곳을 이동한 뒤 rc_pos 목표를 받아 이동하는 노드.
# ros2 topic pub --once /robot2/web_true std_msgs/msg/Bool "{data: true}"

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

INITIAL_POSE_POSITION = [-0.0, 0.0]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

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

        self.web_sub = self.create_subscription(Bool, 'web_true', self.web_callback, 10)
        self.rc_pos_sub = self.create_subscription(
            PointStamped,
            'rc_pos',
            self.rc_pos_callback,
            10,
        )

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

    def rc_pos_callback(self, msg):
        if self.state != 'WAIT_RC_POS':
            self.get_logger().debug('Ignoring rc_pos until waypoint route is completed.')
            return

        self.get_logger().info(
            f'Received rc_pos goal: ({msg.point.x:.2f}, {msg.point.y:.2f})'
        )

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.position.z = 0.0

        yaw = 0.0
        goal_pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

        self.navigator.goToPose(goal_pose)
        self.state = 'WAIT_RC_GOAL'
        self.get_logger().info('Sent navigation goal to rc_pos map coordinate.')


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
