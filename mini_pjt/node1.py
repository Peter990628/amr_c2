# 'web_true' sub 하면 amr 을 goal 지점으로 이동
# 도착하면 'goal' 토픽 pub 하는 노드

import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [-0.0, 0.0]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [
    ([-2.7835135459899902, 3.949268341064453], TurtleBot4Directions.SOUTH),
]
# =====================


class RobotStart(Node):
    def __init__(self):
        super().__init__('robot_start')
        self.is_navigating = False
        self.navigator = TurtleBot4Navigator()
        self.goal_publisher = self.create_publisher(Bool, 'goal', 10)
        self.subscription = self.create_subscription(
            Bool,
            'web_true',
            self.web_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('RobotStart node has been started.')
        self.initialize_navigation()

    def initialize_navigation(self):
        initial_pose = self.navigator.getPoseStamped(
            INITIAL_POSE_POSITION,
            INITIAL_POSE_DIRECTION,
        )
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Initial pose is set to map origin.')

    def web_callback(self, msg):
        if not msg.data:
            return

        if self.is_navigating:
            self.get_logger().info(
                'Navigation is already in progress. Ignoring request.'
            )
            return

        self.is_navigating = True
        navigator = self.navigator

        try:
            if navigator.getDockedStatus():
                navigator.undock()

            goal_pose = navigator.getPoseStamped(*GOAL_POSES[0])
            navigator.startToPose(goal_pose)

            if navigator.getResult() == TaskResult.SUCCEEDED:
                self.goal_publisher.publish(Bool(data=True))
                self.get_logger().info("Goal reached. Published 'goal' topic.")
            else:
                self.get_logger().warn(
                    'Failed to reach goal. Goal topic was not published.'
                )
        finally:
            self.is_navigating = False

        # navigator.dock()


def main():
    rclpy.init()
    node = RobotStart()
    try:
        rclpy.spin(node)
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
