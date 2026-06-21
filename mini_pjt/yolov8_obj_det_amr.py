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
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class YOLOProcessorAMR(Node):
    def __init__(self, model, output_dir,
                 camera_topic='/robot2/oakd/rgb/image_raw/compressed',
                 save_detections=True):
        super().__init__('yolo_processor_amr')
        self.model = model
        self.output_dir = output_dir

        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0

        self.classNames = model.names
        self.should_shutdown = False
        self.bridge = CvBridge()
        self.save_detections = save_detections  # 기본 True로 복원

        # QoS depth 10으로 복원
        self.subscription = self.create_subscription(
            CompressedImage, camera_topic, self.image_callback, 10)

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(img, verbose=False)

        object_count = 0
        fontScale = 1

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                label = self.classNames.get(cls, f"class_{cls}")
                self.confidences.append(confidence)

                cv2.putText(img, f"{label}: {confidence}", (x1, y1),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                self.csv_output.append([x1, y1, x2, y2, confidence, label])
                object_count += 1

        self.max_object_count = max(self.max_object_count, object_count)
        cv2.putText(img, f"Objects_count: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        # 매 프레임 검출 시 디스크 저장 (복원)
        if object_count > 0 and self.save_detections:
            filename = f'output_{int(time.time())}.jpg'
            cv2.imwrite(os.path.join(self.output_dir, filename), img)

        cv2.imshow("Detection", img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Shutting down...")
            self.should_shutdown = True
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def save_output(self):
        with open(os.path.join(self.output_dir, 'output.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['X1', 'Y1', 'X2', 'Y2', 'Confidence', 'Class'])
            writer.writerows(self.csv_output)

        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:   # 추가
            json.dump(self.csv_output, f)                                    # 추가

        with open(os.path.join(self.output_dir, 'statistics.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Max Object Count', 'Average Confidence'])
            avg_conf = sum(self.confidences) / len(self.confidences) if self.confidences else 0
            writer.writerow([self.max_object_count, avg_conf])


def main():
    model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()

    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    rclpy.init()
    processor = YOLOProcessorAMR(model, output_dir)

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        processor.save_output()  # 종료 시 결과 저장 (복원)
        processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown complete.")
        sys.exit(0)


if __name__ == '__main__':
    main()