#  'goal' 토픽 sub 하면 waypoint로 이동하는 노드 1->2->3->2->1 로 이동하는 노드 


import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

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
        self.goal_sub = self.create_subscription(Bool, '/goal', self.goal_callback, 10)
        self.yolo_sub = self.create_subscription(Bool, '/yolo_pos', self.yolo_callback, 10)
        self.center_start_pub = self.create_publisher(Bool, '/center_start', 10) 

        self.timer = self.create_timer(0.2, self.update)
        self.state = 'IDLE'
        self.route_index = 0


    def goal_callback(self, msg):
        if not msg.data or self.state != 'IDLE':
            return

        self.route_index = 0
        self.state = 'SPIN_START'

    def update(self):
        if self.state == 'SPIN_START':
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
                self.route_index += 1
                self.navigator.spin(spin_dist=6.28318, time_allowance=20)
                self.state = 'WAIT_SPIN'

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
