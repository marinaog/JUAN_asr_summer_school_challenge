#!/usr/bin/env python3
"""Accumulate timestamped AprilTag observations and export a semantic map."""
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from asr_summer_school.mission_logic import atomic_yaml, summarize


class TagMapper(Node):
    def __init__(self):
        super().__init__('tag_mapper')
        for name, default in [('output_dir', '~/challenge_results/current'),
                              ('detections_topic', '/camera/detections'),
                              ('map_frame', 'map'), ('tf_wait_sec', 1.0),
                              ('consistency_m', 0.30), ('save_interval_sec', 30.0)]:
            self.declare_parameter(name, default)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=1000)
        self.records = {}
        self.seen = {}
        self.dropped = 0
        self.active = False
        self.recording = False
        self.output = Path(self.get_parameter('output_dir').value).expanduser()
        self.create_subscription(AprilTagDetectionArray,
                                 self.get_parameter('detections_topic').value,
                                 self.detect, 10)
        self.create_service(Trigger, '/tag_mapper/start', self.start)
        self.create_service(Trigger, '/tag_mapper/save', self.save_service)
        self.create_service(Trigger, '/tag_mapper/stop', self.stop)
        self.publisher = self.create_publisher(String, '/mission/tags', 10)
        self.create_timer(0.05, self.process)
        self.create_timer(self.get_parameter('save_interval_sec').value, self.save)
        self.create_timer(1.0, self.status)

    def start(self, request, response):
        if self.active:
            response.message = 'Tag mapper already recording'
            return response
        self.active = True
        self.recording = True
        response.success = True
        response.message = 'Recording observations'
        return response

    def stop(self, request, response):
        self.recording = False
        self.process()
        self.dropped += len(self.pending)
        self.pending.clear()
        response.success = self.save()
        response.message = 'Recording stopped; semantic export attempted'
        return response

    def detect(self, msg):
        if not self.recording:
            return
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        if stamp == 0:
            return  # Time(0) means latest TF, not an observation-time lookup.
        for tag in msg.detections:
            key = (tag.family, tag.id)
            if tag.hamming != 0 or self.seen.get(key) == stamp:
                continue
            self.seen[key] = stamp
            if len(self.pending) == self.pending.maxlen:
                self.dropped += 1
            self.pending.append((time.monotonic(), stamp, tag.family, tag.id))

    def process(self):
        for _ in range(len(self.pending)):
            received, stamp, family, tag_id = self.pending.popleft()
            try:
                tf = self.buffer.lookup_transform(
                    self.get_parameter('map_frame').value, f'{family}:{tag_id}',
                    Time(nanoseconds=stamp))
            except TransformException:
                if time.monotonic()-received < self.get_parameter('tf_wait_sec').value:
                    self.pending.append((received, stamp, family, tag_id))
                else:
                    self.dropped += 1
                continue
            p = tf.transform.translation
            observation = {'stamp_ns': stamp, 'position': [p.x, p.y, p.z]}
            key = f'{family}:{tag_id}'
            record = self.records.setdefault(key, {'family': family, 'id': tag_id,
                                                   'samples': deque(maxlen=300)})
            record['samples'].append(observation)
            # Append all observations for later refinement without unbounded RAM use.
            try:
                self.output.mkdir(parents=True, exist_ok=True)
                with (self.output/'observations.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(observation, family=family, id=tag_id))+'\n')
            except OSError as error:
                self.get_logger().error(f'Observation export failed: {error}')

    def document(self):
        tags = []
        for record in self.records.values():
            samples = list(record['samples'])
            tags.append(dict(family=record['family'], id=record['id'],
                             first_stamp_ns=samples[0]['stamp_ns'],
                             last_stamp_ns=samples[-1]['stamp_ns'],
                             **summarize(samples, self.get_parameter('consistency_m').value)))
        return {'frame_id': self.get_parameter('map_frame').value,
                'tags': tags, 'dropped_observations': self.dropped}

    def status(self):
        tags = self.document()['tags']
        self.publisher.publish(String(data=json.dumps({
            'unique': len(tags), 'confirmed': sum(t['confirmed'] for t in tags)})))

    def save(self):
        if not self.active:
            return False
        try:
            atomic_yaml(self.output/'semantic_map.yaml', self.document())
            return True
        except OSError as error:
            self.get_logger().error(f'Semantic export failed: {error}')
            return False

    def save_service(self, request, response):
        response.success = self.save()
        response.message = 'Saved semantic map' if response.success else 'Semantic export failed'
        return response


def main():
    rclpy.init()
    node = TagMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
