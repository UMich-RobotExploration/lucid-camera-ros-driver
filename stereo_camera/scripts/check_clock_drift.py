#!/usr/bin/env python3
"""Check per-topic clock drift in a recorded bag.

For each topic, compares each message's header.stamp against the bag's own
recording-arrival timestamp (the recording PC's clock at the moment the
message was written -- independent of whatever clock the message itself is
stamped with). A topic whose clock runs at the same rate as the recording
PC's clock will show a flat residual over time; a topic whose clock is
drifting will show a steady slope.

This works even when different topics' header.stamp values sit on
completely different epochs (e.g. an arbitrary PTP domain vs real UTC) --
only the *rate*, not the absolute offset, is meaningful here.

Usage:
    python3 check_clock_drift.py <bag_path> [topic ...]

With no topics given, checks every topic found in the bag.
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topic(bag_path, topic_filter):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    # per-message zstd compression (as used by our record launch file) needs
    # the compression-aware reader -- the plain SequentialReader hands back
    # still-compressed bytes and deserialize_message chokes on them.
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_filter:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topic_filter))

    samples = {}
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if not hasattr(msg, 'header'):
            continue
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        samples.setdefault(topic, []).append((bag_time_ns, stamp_ns))
    return samples


def analyze(topic, pairs):
    if len(pairs) < 10:
        print(f'{topic}: only {len(pairs)} messages, skipping (need >=10)')
        return
    arr = np.array(pairs, dtype=np.float64)
    bag_t = arr[:, 0]
    stamp_t = arr[:, 1]
    residual = stamp_t - bag_t

    t0 = bag_t[0]
    elapsed_s = (bag_t - t0) / 1e9
    duration_s = elapsed_s[-1]
    if duration_s < 1.0:
        print(f'{topic}: recording too short ({duration_s:.1f}s), skipping')
        return

    # linear fit: residual (ns) vs elapsed time (s) -> slope is ns/s drift
    slope_ns_per_s, intercept = np.polyfit(elapsed_s, residual, 1)
    fitted = slope_ns_per_s * elapsed_s + intercept
    jitter_ns = np.std(residual - fitted)

    ppm = slope_ns_per_s / 1e3  # (ns/s) / 1000 = ppm
    print(f'{topic}:')
    print(f'  {len(pairs)} messages over {duration_s:.1f}s')
    print(f'  drift: {slope_ns_per_s:+.2f} ns/s  ({ppm:+.3f} ppm)'
          f'  -> {slope_ns_per_s * 3600 / 1e6:+.3f} ms/hour')
    print(f'  jitter (residual std after de-trending): {jitter_ns/1e3:.2f} us')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag_path = sys.argv[1]
    topics = sys.argv[2:]

    samples = read_topic(bag_path, topics)
    if not samples:
        print('No stamped messages found for the requested topic(s).')
        sys.exit(1)

    for topic, pairs in samples.items():
        analyze(topic, pairs)
        print()


if __name__ == '__main__':
    main()
