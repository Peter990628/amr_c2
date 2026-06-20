# 'web_true' 토픽을 받으면 시작 지점으로 이동한 뒤 waypoint 순찰을 수행하는 노드.
# 'goal' 토픽을 직접 받으면 시작 지점 이동 없이 waypoint 순찰만 수행할 수 있다.


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

POINT_1 = ([-2.7835135459899902, 3.949268341064453], TurtleBot4Directions.SOUTH)
POINT_2 = ([-3.7630257606506348, 3.9203786849975586], TurtleBot4Directions.SOUTH)
POINT_3 = ([-4.553833484649658, 3.8361518383026123], TurtleBot4Directions.SOUTH)

ROUTE = [
    POINT_2,
    POINT_3,
    POINT_2,
    POINT_1,
]


class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')
        self.navigator = TurtleBot4Navigator()
        self.initialize_navigation()
        self.web_sub = self.create_subscription(Bool, 'web_true', self.web_callback, 10)
        # self.goal_sub = self.create_subscription(Bool, 'goal', self.goal_callback, 10)
        self.yolo_sub = self.create_subscription(Bool, 'yolo_pos', self.yolo_callback, 10)
        self.center_start_pub = self.create_publisher(Bool, 'center_start', 10)

        self.timer = self.create_timer(0.2, self.update)
        self.state = 'IDLE'
        self.route_index = 0

        self.get_logger().info('Waypoint node has been started.')

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

        self.state = 'START_GOAL'
        self.get_logger().info('Web trigger received. Moving to first goal.')

    def goal_callback(self, msg):
        if not msg.data or self.state != 'IDLE':
            return

        self.start_patrol()

    def start_patrol(self):
        self.route_index = 0
        self.state = 'SPIN_START'
        self.get_logger().info('Patrol started.')

    def update(self):
        if self.state == 'START_GOAL':
            if self.navigator.getDockedStatus():
                self.navigator.undock()

            position, direction = POINT_1
            pose = self.navigator.getPoseStamped(position, direction)
            self.navigator.goToPose(pose)
            self.state = 'WAIT_START_GOAL'

        elif self.state == 'WAIT_START_GOAL':
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('First goal reached. Starting patrol.')
                    self.start_patrol()
                else:
                    self.get_logger().warn('Failed to reach first goal.')
                    self.state = 'IDLE'

        elif self.state == 'SPIN_START':
            self.navigator.spin(spin_dist=6.28318, time_allowance=20)
            self.state = 'WAIT_SPIN'

        elif self.state == 'WAIT_SPIN':
            if self.navigator.isTaskComplete():
                self.state = 'GO_NEXT'

        elif self.state == 'GO_NEXT':
            if self.route_index >= len(ROUTE):
                self.state = 'IDLE'
                return

            position, direction = ROUTE[self.route_index]
            pose = self.navigator.getPoseStamped(position, direction)
            self.navigator.goToPose(pose)
            self.state = 'WAIT_GOAL'

        elif self.state == 'WAIT_GOAL':
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.route_index += 1
                    self.navigator.spin(spin_dist=6.28318, time_allowance=20)
                    self.state = 'WAIT_SPIN'
                else:
                    self.get_logger().warn('Failed to reach patrol waypoint.')
                    self.state = 'IDLE'

    def yolo_callback(self, msg):
        if not msg.data:
            return

        if self.state != 'IDLE':
            self.navigator.cancelTask()
            self.state = 'YOLO_INTERRUPTED'
            self.center_start_pub.publish(Bool(data=True))
            self.get_logger().info('YOLO detected. Patrol canceled. Centering started.')


def main():
    rclpy.init()
    node = WaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
