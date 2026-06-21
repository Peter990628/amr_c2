import json
import csv
import time
import math
import os
import shutil
import sys
from pathlib import Path

import cv2
import rclpy

from ultralytics import YOLO

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class YOLOWebcamPublisher(Node):

    def __init__(self, model, output_dir):
        super().__init__('yolo_webcam_publisher')
        self.model = model
        self.output_dir = output_dir
        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            'processed_image',
            10
        )

        self.red_car_pub = self.create_publisher(
            Bool,
            'web_true',
            10
        )

        self.red_car_confidences = []
        self.amr_start = False
        self.should_shutdown = False
        self.publish_count = 0
        self.delay_timer = None
        self.publish_timer = None
        self.shutdown_timer = None

        self.cap = cv2.VideoCapture(1)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            raise RuntimeError(
                "Webcam not available"
            )

        self.timer = self.create_timer(
            0.1,
            self.process_frame
        )

    def process_frame(self):
        if self.should_shutdown:
            return
        ret, img = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam.")
            return

        results = self.model(
            img,
            stream=True
        )

        object_count = 0
        fontScale = 1
        current_confidence = 0.0

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(
                    int, box.xyxy[0]
                )
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                confidence = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.classNames.get(cls, f"class_{cls}")
                self.confidences.append(confidence)

                cv2.putText(
                    img, f"{label}: {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)
                self.csv_output.append([x1, y1, x2, y2, confidence, label])

                object_count += 1
                # class 0 = red_car
                if cls == 0:
                    current_confidence = max(current_confidence, confidence)

        # red car 검증
        if current_confidence >= 0.8:
            self.red_car_confidences.append(
                current_confidence
            )
            self.get_logger().info(
                f"red_car detected "
                f"({len(self.red_car_confidences)}/50) "
                f"conf={current_confidence:.2f}"
            )

        else:
            if len(self.red_car_confidences) > 0:
                self.get_logger().info("Detection reset")
            self.red_car_confidences.clear()

        # red_car 연속 검출 성공
        if (len(self.red_car_confidences) >= 50 and not self.amr_start):
            self.amr_start = True
            self.get_logger().info("RED CAR VERIFIED")
            self.delay_timer = self.create_timer(
                3.0,
                self.start_publish_timer
            )
            return

        self.max_object_count = max(
            self.max_object_count,
            object_count
        )

        cv2.putText(
            img,
            f"Objects_count: {object_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale,
            (0, 255, 0),
            1
        )

        if object_count > 0:
            filename = (
                f'output_{int(time.time())}.jpg'
            )
            cv2.imwrite(
                os.path.join(
                    self.output_dir,
                    filename
                ),
                img
            )

        # 화면 표시
        display_img = cv2.resize(
            img,
            (img.shape[1] * 2,
             img.shape[0] * 2))
        print("SHOW WINDOW")

        cv2.imshow(
            "Detection",
            display_img
        )

        key = cv2.waitKey(10)
        if key == ord('q'):

            self.get_logger().info(
                "q pressed. Shutting down..."
            )
            self.should_shutdown = True
            return

 
        # ROS Image Publish
        msg = self.bridge.cv2_to_imgmsg(
            display_img,
            encoding="bgr8"
        )
        self.publisher.publish(msg)


    def start_publish_timer(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
        self.get_logger().info("Start publishing True")
        self.publish_count = 0
        self.publish_timer = self.create_timer(
            0.1,
            self.publish_true_callback
        )


    def publish_true_callback(self):
        bool_msg = Bool()
        bool_msg.data = True
        self.red_car_pub.publish(bool_msg)
        self.publish_count += 1
        self.get_logger().info(
            f"Publish True "
            f"({self.publish_count}/10)"
            )

        if self.publish_count >= 10:
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            self.get_logger().info("Finished publishing True")
            self.shutdown_timer = self.create_timer(1.0, self.shutdown_callback)


    def shutdown_callback(self):
        self.get_logger().info("Shutting down node...")
        self.should_shutdown = True
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()

    def save_output(self):
        with open(
            os.path.join(
                self.output_dir,
                'output.csv'
            ),
            'w',
            newline=''
        ) as f:
            writer = csv.writer(f)
            writer.writerow([
                'X1', 'Y1', 'X2', 'Y2', 'Confidence', 'Class'
            ])
            writer.writerows(
                self.csv_output
            )
        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:
            json.dump(
                self.csv_output,
                f
            )
        with open(
            os.path.join(self.output_dir, 'statistics.csv'), 'w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Max Object Count',
                'Average Confidence'
            ])
            avg_conf = (
                sum(self.confidences) / len(self.confidences)
                if self.confidences
                else 0
            )
            writer.writerow([
                self.max_object_count,
                avg_conf
            ])

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    model_path = input(
        "Enter path to model file "
        "(.pt, .engine, .onnx): "
    ).strip()

    if not os.path.exists(model_path):
        print(
            f"File not found: "
            f"{model_path}"
        )
        sys.exit(1)

    suffix = (
        Path(model_path)
        .suffix
        .lower()
    )

    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(
            f"Unsupported "
            f"model format: "
            f"{suffix}"
        )
        sys.exit(1)

    output_dir = './output'

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    os.mkdir(output_dir)

    rclpy.init()

    node = YOLOWebcamPublisher(
        model,
        output_dir
    )

    try:
        while (rclpy.ok() and not node.should_shutdown):
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("Ctrl+C Shutdown complete.")

    finally:
        node.save_output()
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()