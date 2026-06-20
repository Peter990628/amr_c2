#  'goal' 토픽 sub 하면 waypoint로 이동하는 노드 1->2->3->2->1 로 이동하는 노드 


import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

class GoalSubnode(Node):
    def __init__(self):
        super().__init__('goal_subnode')
        self.is_navigating = False
        self.subscriptions = self.create_subscription(Bool, '/goal', self.goal_callback, 10)



    def goal_callback(self, msg):
        if not msg.data:
            return

        if self.is_navigating:
            self.get_logger().info(
                'Navigation is already in progress. Ignoring request.'
            )
            return


        self.is_navigating = True
        navigator = TurtleBot4Navigator()

        try:
            # # Set initial pose
            # initial_pose = navigator.getPoseStamped([-2.7835135459899902, 3.949268341064453], TurtleBot4Directions.SOUTH)
            # navigator.setInitialPose(initial_pose)

            # Wait for Nav2
            navigator.waitUntilNav2Active()

            # Set goal poses
            goal_pose = []
            # goal_pose.append(navigator.getPoseStamped([-3.3, 5.9], TurtleBot4Directions.NORTH))
            # goal_pose.append(navigator.getPoseStamped([2.1, 6.3], TurtleBot4Directions.EAST))
            # goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
            # goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

            # goal_pose.append(navigator.getPoseStamped([-2.7835135459899902, 3.949268341064453], TurtleBot4Directions.SOUTH))
            goal_pose.append(navigator.getPoseStamped([-3.7630257606506348, 3.9203786849975586], TurtleBot4Directions.SOUTH))
            goal_pose.append(navigator.getPoseStamped([-4.553833484649658, 3.8361518383026123], TurtleBot4Directions.SOUTH))
        
            # Follow Waypoints
            navigator.startFollowWaypoints(goal_pose)
             # 로봇 한바퀴 돌아가는 코드 있으면 좋을 듯 

        finally:
            navigator.destroyNode()
            self.is_navigating = False



def main():
    rclpy.init()

    # navigator = TurtleBot4Navigator()

    # # Start on dock
    # if not navigator.getDockedStatus():
    #     navigator.info('Docking before intialising pose')
    #     navigator.dock()

    # # Set initial pose
    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    # # Wait for Nav2
    # navigator.waitUntilNav2Active()

    # # Set goal poses
    # goal_pose = []
    # # goal_pose.append(navigator.getPoseStamped([-3.3, 5.9], TurtleBot4Directions.NORTH))
    # # goal_pose.append(navigator.getPoseStamped([2.1, 6.3], TurtleBot4Directions.EAST))
    # # goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
    # # goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

    # goal_pose.append(navigator.getPoseStamped([-2.7835135459899902, 3.949268341064453], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-3.7630257606506348, 3.9203786849975586], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-4.553833484649658, 3.8361518383026123], TurtleBot4Directions.SOUTH))
    # # goal_pose.append(navigator.getPoseStamped([-0.711899, -0.0612125], TurtleBot4Directions.NORTH))

    # # Undock
    # navigator.undock()

    # # Follow Waypoints
    # navigator.startFollowWaypoints(goal_pose)

    # # Finished navigating, dock
    # # navigator.dock()
    node = GoalSubnode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
