"""Passively compare raw video, displayed annotations and robot observations.

No SDK connection, image conversion, detector or command publisher. Driver image
stamps denote driver receipt/decode timing, not camera hardware exposure time.
"""
import argparse
import json
import math
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String


class StreamTiming:
    def __init__(self):
        self.arrivals = []
        self.ages = []
        self.stamps = []

    def record(self, stamp, now_ros, now_wall):
        self.arrivals.append(now_wall)
        self.stamps.append(stamp)
        self.ages.append(now_ros-stamp)

    def summary(self):
        if not self.arrivals:
            return dict(messages=0)
        intervals = [b-a for a, b in zip(self.arrivals, self.arrivals[1:])]
        stamp_gaps = [b-a for a, b in zip(self.stamps, self.stamps[1:]) if b > a]
        return dict(messages=len(self.arrivals),
                    arrival_hz=(len(intervals)/(self.arrivals[-1]-self.arrivals[0])
                                if intervals and self.arrivals[-1] > self.arrivals[0] else None),
                    median_arrival_gap_s=statistics.median(intervals) if intervals else None,
                    max_arrival_gap_s=max(intervals) if intervals else None,
                    median_header_age_s=statistics.median(self.ages),
                    max_header_age_s=max(self.ages),
                    max_source_stamp_gap_s=max(stamp_gaps) if stamp_gaps else None)


class VideoDiagnostics(Node):
    def __init__(self, annotated_transport='compressed'):
        super().__init__('phase3_video_diagnostics')
        self.streams = {name: StreamTiming() for name in ('raw', 'annotated', 'control_observation')}
        self.dimensions = {}
        self.paired = {}
        self.annotation_delay = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_color', lambda msg: self.image('raw', msg), qos)
        message_type = CompressedImage if annotated_transport == 'compressed' else Image
        topic = '/camera/annotated/compressed' if annotated_transport == 'compressed' else '/camera/annotated'
        self.create_subscription(message_type, topic, lambda msg: self.image('annotated', msg), qos)
        self.create_subscription(String, '/phase3/camera_observation', self.observation, qos)

    def record(self, name, stamp):
        wall = time.monotonic()
        self.streams[name].record(stamp, self.get_clock().now().nanoseconds*1e-9, wall)
        return wall

    def image(self, name, msg):
        stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        wall = self.record(name, stamp_ns*1e-9)
        if hasattr(msg, 'width'):
            self.dimensions[name] = [msg.width, msg.height]
        pair = self.paired.setdefault(stamp_ns, {})
        pair[name] = wall
        if 'raw' in pair and 'annotated' in pair:
            self.annotation_delay.append(pair['annotated']-pair['raw'])
            del self.paired[stamp_ns]
        if len(self.paired) > 512:
            del self.paired[next(iter(self.paired))]

    def observation(self, msg):
        try:
            stamp = float(json.loads(msg.data)['stamp_s'])
        except (ValueError, KeyError, TypeError):
            return
        if math.isfinite(stamp):
            self.record('control_observation', stamp)

    def report(self):
        result = {name: stream.summary() for name, stream in self.streams.items()}
        result['dimensions_px'] = self.dimensions
        delays = self.annotation_delay
        result['annotation_minus_raw_receipt_s'] = dict(
            matched_frames=len(delays), median=statistics.median(delays) if delays else None,
            maximum=max(delays) if delays else None)
        result['interpretation'] = [
            'Raw arrival gaps localize stalls before this observer receives driver images.',
            'Fresh raw but delayed annotation localizes stalls to camera processing/publication.',
            'Fresh raw and annotated but frozen window points to viewer/display scheduling.',
            'Control uses control_observation timestamps, not GUI repaint timing.',
            'This checks delivered ROS images, not hardware camera exposure latency.',
            'This observer adds best-effort image subscriptions; compare with existing logs if load-sensitive.',
        ]
        print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--annotated-transport', choices=['compressed', 'raw'], default='compressed')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    rclpy.init(args=ros_args)
    node = VideoDiagnostics(args.annotated_transport)
    deadline = time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
