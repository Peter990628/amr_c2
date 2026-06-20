import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriberAMR(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_amr')
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_frame = None

    def listener_callback(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberAMR()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.last_frame is not None:
                # display = cv2.resize(node.last_frame, None, fx=1.5, fy=1.5)
                # cv2.imshow("Processed Image", display)
                cv2.imshow("Processed Image", node.last_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("🛑 'q' pressed, exiting...")
                    break

    finally:
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ Subscriber shutdown complete.")

if __name__ == '__main__':
    main()
