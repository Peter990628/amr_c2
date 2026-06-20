import json
import csv
import time
import math
import os
import shutil
import sys
from ultralytics import YOLO
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class YOLOPublisherAMR(Node):
    def __init__(self, model, output_dir, camera_topic='/robot2/oakd/rgb/image_raw'):
        super().__init__('yolo_publisher_amr')
        self.model = model
        self.output_dir = output_dir
        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'processed_image', 10)
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'yolo_pos_amr', 10) # 추가
        self.should_shutdown = False

        # 웹캠(cv2.VideoCapture) 대신 AMR 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image, camera_topic, self.process_frame, 10)

    def process_frame(self, msg):
        if self.should_shutdown:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(img, stream=True)
        object_count = 0
        fontScale = 1

        best_box = None
        best_conf = -1.0

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                label = self.classNames.get(cls, f"class_{cls}")
                self.confidences.append(confidence)

                org = [x1, y1]
                cv2.putText(img, f"{label}: {confidence}", org,
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                self.csv_output.append([x1, y1, x2, y2, confidence, label])
                object_count += 1

                # 가장 신뢰도 높은 박스 추적
                if confidence > best_conf:
                    best_conf = confidence
                    best_box = (x1, y1, x2, y2)


        # ---- yolo_pos 퍼블리시 ----
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            pos_msg = Float32MultiArray()
            pos_msg.data = [float(x1), float(y1), float(x2), float(y2), cx, cy]
            self.pos_publisher.publish(pos_msg)
        # --------------------------

        self.max_object_count = max(self.max_object_count, object_count)
        cv2.putText(img, f"Objects_count: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        if object_count > 0:
            filename = f'output_{int(time.time())}.jpg'
            cv2.imwrite(os.path.join(self.output_dir, filename), img)

        display_img = cv2.resize(img, (img.shape[1] * 2, img.shape[0] * 2))
        out_msg = self.bridge.cv2_to_imgmsg(display_img, encoding="bgr8")
        out_msg.header = msg.header
        self.publisher.publish(out_msg)

    def save_output(self):
        with open(os.path.join(self.output_dir, 'output.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['X1', 'Y1', 'X2', 'Y2', 'Confidence', 'Class'])
            writer.writerows(self.csv_output)

        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:
            json.dump(self.csv_output, f)

        with open(os.path.join(self.output_dir, 'statistics.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Max Object Count', 'Average Confidence'])
            avg_conf = sum(self.confidences) / len(self.confidences) if self.confidences else 0
            writer.writerow([self.max_object_count, avg_conf])

    def destroy_node(self):
        super().destroy_node()

def main():
    model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()

    if not os.path.exists(model_path):
        print(f"❌ File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"❌ Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    rclpy.init()
    node = YOLOPublisherAMR(model, output_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔴 Ctrl+C received. Exiting...")
    finally:
        node.save_output()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
