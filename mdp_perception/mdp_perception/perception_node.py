import json
from collections import Counter, defaultdict
from pathlib import Path
from threading import Event, Lock, Thread

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from greenhouse_sim.simulator import GreenhouseSimulator
from pupil_apriltags import Detector
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ultralytics import YOLO

TOPIC_IMAGE = '/gripper_camera/image_raw'
TOPIC_RESULT = '/perception/scan_result'
TOPIC_DEBUG = '/perception/debug_image/compressed'
YOLO_MODEL = str(Path(get_package_share_directory('mdp_perception')) / 'models' / 'flower.pt')
YOLO_CONF = 0.15
YOLO_IOU = 0.4


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('device', 'cpu')

        self._device = self.get_parameter('device').get_parameter_value().string_value

        self._yolo = YOLO(YOLO_MODEL)
        self._tag_detector = Detector(families='tag36h11')
        self._sim = None

        self._scan_lock = Lock()
        self._buffering = False
        self._flower_tracks = defaultdict(set)
        self._bug_tracks = set()
        self._tag_counter = Counter()

        self._latest_frame = None
        self._frame_lock = Lock()
        self._new_frame = Event()

        self._result_pub = self.create_publisher(String, TOPIC_RESULT, 10)
        self._debug_pub = self.create_publisher(CompressedImage, TOPIC_DEBUG, 10)
        self.create_subscription(Image, TOPIC_IMAGE, self._image_cb, 10)

        self.create_service(Trigger, '/perception/start_scan', self._start_cb)
        self.create_service(Trigger, '/perception/stop_scan', self._stop_cb)

        Thread(target=self._inference_loop, daemon=True).start()
        self.get_logger().info('PerceptionNode ready.')

    def _start_scan(self):
        with self._scan_lock:
            self._flower_tracks = defaultdict(set)
            self._bug_tracks = set()
            self._tag_counter = Counter()
            self._buffering = True

    def _start_cb(self, request, response):
        self._start_scan()
        self.get_logger().info('Scan started.')
        response.success = True
        response.message = 'Scan started'
        return response

    def _stop_cb(self, request, response):
        with self._scan_lock:
            if not self._buffering:
                response.success = False
                response.message = 'No scan in progress'
                return response
            self._buffering = False
            tag_id = self._tag_counter.most_common(1)[0][0] if self._tag_counter else None
            flowers = {colour: len(ids) for colour, ids in self._flower_tracks.items()}
            bugs = len(self._bug_tracks)

        conditions = {}
        if tag_id:
            try:
                if self._sim is None:
                    self._sim = GreenhouseSimulator()
                conditions = self._sim.get_sensor_data(tag_id)
            except Exception as e:
                self.get_logger().warn(f'Could not get sensor data for tag {tag_id}: {e}')

        result = {
            'tag_id': tag_id,
            'flowers': flowers,
            'total_flowers': sum(flowers.values()),
            'bugs': bugs,
            'conditions': conditions,
        }
        self._result_pub.publish(String(data=json.dumps(result)))
        self.get_logger().info(
            f'Scan complete — tray {tag_id}: {sum(flowers.values())} flowers, {bugs} bugs'
        )
        response.success = True
        response.message = json.dumps(result)
        return response

    def _decode(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return cv2.rotate(frame, cv2.ROTATE_180)

    def _image_cb(self, msg):
        frame = self._decode(msg)
        with self._frame_lock:
            self._latest_frame = frame
        self._new_frame.set()

    def _inference_loop(self):
        while rclpy.ok():
            self._new_frame.wait()
            self._new_frame.clear()

            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                continue

            try:
                annotated = frame.copy()

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = self._tag_detector.detect(gray)
                tag_id = None
                if detections:
                    best = max(detections, key=lambda d: d.decision_margin)
                    tag_id = str(best.tag_id)
                    corners = best.corners.astype(int)
                    cv2.polylines(annotated, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.putText(annotated, f'tag:{tag_id}', (corners[0][0], corners[0][1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                results = self._yolo.track(
                    frame, conf=YOLO_CONF, iou=YOLO_IOU,
                    device=self._device, persist=True, verbose=False
                )

                for r in results:
                    if r.boxes.id is None:
                        continue
                    for box, track_id in zip(r.boxes, r.boxes.id.int().tolist()):
                        label = self._yolo.names[int(box.cls[0])]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0])
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 120, 255), 2)
                        cv2.putText(annotated, f'{label} {conf:.2f} #{track_id}', (x1, y1 - 6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 255), 1)

                        with self._scan_lock:
                            if not self._buffering:
                                continue
                            if label == 'bug':
                                self._bug_tracks.add(track_id)
                            else:
                                self._flower_tracks[label.split('_', 1)[-1]].add(track_id)

                if tag_id:
                    with self._scan_lock:
                        if self._buffering:
                            self._tag_counter[tag_id] += 1

                _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                out = CompressedImage()
                out.header.stamp = self.get_clock().now().to_msg()
                out.format = 'jpeg'
                out.data = buf.tobytes()
                self._debug_pub.publish(out)

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
