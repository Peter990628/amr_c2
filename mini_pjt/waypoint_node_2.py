# waypoint_node_2.py
# 'web_true' 토픽을 받으면 points.txt 기준 waypoint 3곳을 순서대로 이동하는 노드.
# ros2 topic pub --once /robot2/web_true std_msgs/msg/Bool "{data: true}"

import rclpy
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
POINT_3 = ([-3.5370280742645264, 3.9416348934173584], TurtleBot4Directions.SOUTH)

ROUTE = [
    POINT_1,
    POINT_2,
    POINT_3,
]


class WaypointNode2(Node):
    def __init__(self):
        super().__init__('waypoint_node_2')
        self.navigator = TurtleBot4Navigator()
        self.initialize_navigation()

        self.web_sub = self.create_subscription(Bool, 'web_true', self.web_callback, 10)
        self.yolo_sub = self.create_subscription(Bool, 'yolo_pos', self.yolo_callback, 10)

        self.timer = self.create_timer(0.2, self.update)
        self.state = 'IDLE'
        self.route_index = 0

        self.get_logger().info('Waypoint node 2 has been started.')

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
        self.state = 'GO_NEXT'
        self.get_logger().info('Web trigger received. Starting waypoint route.')

    def update(self):
        if self.state == 'GO_NEXT':
            if self.route_index >= len(ROUTE):
                self.state = 'IDLE'
                self.get_logger().info('Waypoint route completed.')
                return

            position, direction = ROUTE[self.route_index]
            pose = self.navigator.getPoseStamped(position, direction)
            self.navigator.goToPose(pose)
            self.get_logger().info(
                f'Moving to waypoint {self.route_index + 1}/{len(ROUTE)}.'
            )
            self.state = 'WAIT_GOAL'

        elif self.state == 'WAIT_GOAL':
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.route_index += 1
                    self.state = 'GO_NEXT'
                else:
                    self.get_logger().warn('Failed to reach waypoint.')
                    self.state = 'IDLE'

    def yolo_callback(self, msg):
        if not msg.data:
            return

        if self.state != 'IDLE':
            self.navigator.cancelTask()
            self.state = 'YOLO_INTERRUPTED'
            self.get_logger().info('YOLO detected. Waypoint route canceled.')


def main():
    rclpy.init()
    node = WaypointNode2()
    try:
        rclpy.spin(node)
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
