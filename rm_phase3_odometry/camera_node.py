"""Phase 2 camera callbacks, isolated from EKF and disk logging.

CameraInfo, detector, throttle, latest-TF lookup and image_view path match Phase 2.
Only the chassis/FSM side effects are replaced by a small observation message.
This process never publishes cmd_vel and never creates an SDK connection.
"""
import json
import math
import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, Range, CompressedImage
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from .tag_detector import TagDetector

def quaternion_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    return np.array([
        [
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w)
        ],
        [
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w)
        ],
        [
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y)
        ]
    ], dtype=float)


class Phase2Camera(Node):
    def __init__(self):
        super().__init__('phase3_camera')
        param = self.declare_parameter
        self.target_id = int(param('tag_id', 3).value)
        self.tag_family = str(param('tag_family', 'tag36h11').value)
        self.tag_size = float(param('tag_size_m', 0.15).value)
        self.camera_timeout = float(param('camera_timeout_s', 1.0).value)
        self.process_interval = float(param('min_image_interval_s', 0.08).value)
        self.base_frame = str(param('base_frame', 'base_link').value)
        self.camera_frame = 'camera_optical_link'
        self.lost_timeout = 0.45
        self.arrival_width_min = float(param('arrival_width_fraction', 0.20).value)
        self.bridge = CvBridge()
        self.detector = TagDetector(self.tag_family, self.target_id, self.tag_size)
        self.K = self.distortion = None
        self.camera_time = self.last_processed = self.detection_time = -1e9
        self.target_distance = float('inf')
        self.target_bearing = 0.0
        self.bbox_w = self.bbox_h = 0.0
        self.center_x_error = self.center_y_error = float('inf')
        self.tof_distance = None
        self.arm_verified = False
        self.arm_x = float(param('arm_x_m', 0.105).value)
        self.arm_z = float(param('arm_z_m', 0.080).value)
        self.arm_tolerance = float(param('arm_tolerance_m', 0.003).value)
        self.navigation_state = 'WAITING'
        self.tf_ok = False
        self.r_base_camera = None
        self.t_base_camera = np.zeros(3)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.annotated_pub = self.create_publisher(Image, 'camera/annotated', 1)
        self.compressed_pub = self.create_publisher(CompressedImage, 'camera/annotated/compressed', 1)
        self.jpeg_quality = int(param('preview_jpeg_quality', 80).value)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('preview_jpeg_quality must be between 1 and 100')
        self.observation_pub = self.create_publisher(String, 'phase3/camera_observation', 1)
        self.create_subscription(Image, str(param('image_topic', 'camera/image_color').value),
                                 self.on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, str(param('camera_info_topic', 'camera/camera_info').value),
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(PointStamped, str(param('arm_topic', 'arm_position').value),
                                 self.on_arm, qos_profile_sensor_data)
        self.create_subscription(Range, str(param('tof_topic', 'range_0').value),
                                 self.on_tof, qos_profile_sensor_data)
        self.create_subscription(String, 'phase3/navigation_state', self.on_state, 1)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.navigation_state = msg.data

    def on_arm(self, msg):
        self.arm_verified = (msg.header.frame_id == 'arm_base_link' and
            math.isfinite(msg.point.x) and math.isfinite(msg.point.z) and
            abs(msg.point.x-self.arm_x) <= self.arm_tolerance+1e-9 and
            abs(msg.point.z-self.arm_z) <= self.arm_tolerance+1e-9)

    def on_tof(self, msg):
        self.tof_distance = (float(msg.range) if math.isfinite(msg.range) and
                             msg.min_range <= msg.range < msg.max_range else None)

    def publish_observation(self, stamp, target):
        # Internal Phase 3 interface, not an additional driver sensor or image stream.
        finite_distance = (target is not None and target.tvec is not None and
                           self.arm_verified and self.tf_ok and math.isfinite(self.target_distance))
        payload = dict(stamp_s=stamp, tag_id=self.target_id, detected=target is not None,
                       bearing_rad=self.target_bearing if target is not None else None,
                       distance_m=self.target_distance if finite_distance else None,
                       bbox_width_fraction=self.bbox_w if target is not None else 0.,
                       calibrated=self.K is not None)
        self.observation_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def publish_preview(self, overlay, header):
        """Encode only the viewed preview; detection always uses the original image."""
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            annotated.header = header
            self.annotated_pub.publish(annotated)
        if self.compressed_pub.get_subscription_count() > 0:
            ok, encoded = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                compressed = CompressedImage()
                compressed.header = header
                compressed.format = 'bgr8; jpeg compressed bgr8'
                compressed.data = encoded.tobytes()
                self.compressed_pub.publish(compressed)

    def on_camera_info(self, msg):
        K = np.array(
            msg.k,
            dtype=float
        ).reshape(3, 3)

        if (
            K[0, 0] <= 0
            or K[1, 1] <= 0
            or not np.isfinite(K).all()
        ):
            self.K = None
            return

        self.K = K

        self.distortion = (
            np.array(msg.d, dtype=float)
            if msg.d
            else None
        )

        self.camera_frame = (
            msg.header.frame_id or self.camera_frame
        )


    def camera_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time()
            )

            stamp = (
                rclpy.time.Time.from_msg(
                    trans.header.stamp
                ).nanoseconds * 1e-9
            )

            if (
                stamp > 0
                and abs(self.now() - stamp)
                > self.lost_timeout
            ):
                self.tf_ok = False
                return

            self.r_base_camera = quaternion_rotation(
                trans.transform.rotation
            )

            pos = trans.transform.translation

            self.t_base_camera = np.array(
                [pos.x, pos.y, pos.z],
                dtype=float
            )

            self.tf_ok = True

        except tf2_ros.TransformException:
            self.tf_ok = False


    def on_image(self, msg):
        stamp = (
            rclpy.time.Time.from_msg(
                msg.header.stamp
            ).nanoseconds * 1e-9
        )

        now = self.now()

        if not math.isfinite(stamp) or stamp <= 0 or abs(now - stamp) > self.camera_timeout:
            self.get_logger().warn(
                f'Dropping stale camera frame: age {now - stamp:.3f} s '
                f'(camera timeout {self.camera_timeout:.3f} s)',
                throttle_duration_sec=1.0)
            return

        # Every accepted frame counts, including frames skipped by throttling.
        self.camera_time = stamp

        if stamp - self.last_processed < self.process_interval:
            return

        self.last_processed = stamp

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.camera_transform()

            detections = self.detector.detect_all(
                frame,
                self.K,
                self.distortion
            )

            target = next(
                (
                    d
                    for d in detections
                    if d.tag_id == self.target_id
                ),
                None
            )

            # Pixel coverage is usable even when PnP or TF cannot produce a pose.
            if target is not None:
                self.bbox_w = target.bbox_w_fraction
                self.bbox_h = target.bbox_h_fraction
                height, width = frame.shape[:2]
                midpoint = (target.corners.min(axis=0) + target.corners.max(axis=0)) / 2
                self.center_x_error = float(midpoint[0] - width // 2)
                self.center_y_error = float(midpoint[1] - height // 2)
                focal = float(self.K[0, 0]) if self.K is not None else float(width)
                self.target_bearing = math.atan2(width / 2 - midpoint[0], focal)
                if self.arm_verified:
                    self.detection_time = stamp
            else:
                self.detection_time = -1e9

            if (
                self.arm_verified
                and target is not None
                and target.tvec is not None
                and self.tf_ok
            ):
                p = (
                    self.r_base_camera @ target.tvec
                    + self.t_base_camera
                )

                if (
                    np.isfinite(p).all()
                    and p[0] > 0
                ):
                    self.target_distance = float(
                        math.hypot(p[0], p[1])
                    )

                    self.bbox_w = target.bbox_w_fraction
                    self.bbox_h = target.bbox_h_fraction
                    self.detection_time = stamp

            visible = (
                now - self.detection_time
            ) <= self.lost_timeout

            self.publish_observation(stamp, target)

            overlay = self.detector.annotate(
                frame,
                detections,
                self.target_id,
                self.navigation_state if self.arm_verified else 'ARM SETUP',
                (
                    self.target_distance
                    if visible
                    else None
                ),
                self.tof_distance,
                self.arrival_width_min
            )

            self.publish_preview(overlay, msg.header)

        except Exception as exc:
            self.get_logger().warn(
                f'OpenCV/TF frame processing error: {exc}',
                throttle_duration_sec=2.0
            )



def main(args=None):
    rclpy.init(args=args)
    node = Phase2Camera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
