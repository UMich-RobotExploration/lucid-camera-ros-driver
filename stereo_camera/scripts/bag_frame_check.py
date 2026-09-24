#!/usr/bin/env python3
"""Validate a recorded stereo bag: per-camera frame counts, PTP-stamp gaps, and
left/right pairing — the bag-side counterpart of the node's [health] counters.

    python3 bag_frame_check.py ~/bags/stereo_lidar_20260924_162608 [--fps 19.7] [--tolerance-us 5000]

Pass criterion for a recording: no gaps on either camera and every left frame
paired with a right frame. Compare "in bag" with the node's final
delivered/published counts to see whether anything was lost between the
publisher and the bag."""
import argparse
import os
import struct
import sys

import yaml


def stamp_from_cdr(data):
    """Header.stamp is the first field of sensor_msgs/Image: 4-byte CDR
    encapsulation header, then int32 sec + uint32 nanosec."""
    little = data[1] in (0x01, 0x03)
    sec, nsec = struct.unpack(('<' if little else '>') + 'iI', data[4:12])
    return sec * 1_000_000_000 + nsec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--fps', type=float, default=19.7, help='PTPSyncFrameRate used for the recording')
    ap.add_argument('--tolerance-us', type=float, default=5000.0)
    ap.add_argument('--left', default='/camera/left/image_raw')
    ap.add_argument('--right', default='/camera/right/image_raw')
    args = ap.parse_args()

    import rosbag2_py
    meta = yaml.safe_load(open(os.path.join(args.bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    compressed = bool(meta.get('compression_format'))
    reader = rosbag2_py.SequentialCompressionReader() if compressed else rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=meta['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (args.left, args.right):
        if t not in topics:
            sys.exit(f"topic {t} not in bag (have: {sorted(topics)})")
        if topics[t] != 'sensor_msgs/msg/Image':
            sys.exit(f"{t} is {topics[t]}, expected sensor_msgs/msg/Image")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.left, args.right]))

    stamps = {args.left: [], args.right: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        stamps[topic].append(stamp_from_cdr(data))

    period = 1e9 / args.fps
    tol = args.tolerance_us * 1000
    print(f"bag: {args.bag}  fps={args.fps}  tolerance={args.tolerance_us:.0f}µs")
    ok = True
    for name, t in (('left', args.left), ('right', args.right)):
        s = stamps[t]
        if not s:
            print(f"  {name}: NO FRAMES"); ok = False; continue
        srt = sorted(s)
        span = (srt[-1] - srt[0]) / 1e9
        expected = round(span * args.fps) + 1
        gaps = dups = back = 0
        for a, b in zip(s, s[1:]):
            k = round((b - a) / period)
            if k > 1: gaps += k - 1
            elif k == 0: dups += 1
            elif k < 0: back += 1
        print(f"  {name}: in bag={len(s)} span={span:.1f}s expected@fps={expected} "
              f"missing(gaps)={gaps} duplicates={dups} out_of_order={back} "
              f"first={srt[0]} last={srt[-1]}")
        ok &= gaps == 0 and dups == 0 and back == 0

    # pair by PTP stamp (same merge as the node's sync monitor)
    L, R = sorted(stamps[args.left]), sorted(stamps[args.right])
    i = j = matched = 0; unl = unr = 0; dmax = 0; dsum = 0
    while i < len(L) and j < len(R):
        d = L[i] - R[j]
        if abs(d) <= tol:
            matched += 1; dsum += abs(d); dmax = max(dmax, abs(d)); i += 1; j += 1
        elif d < 0:
            unl += 1; i += 1
        else:
            unr += 1; j += 1
    unl += len(L) - i; unr += len(R) - j
    # Frames outside the window both cameras were recording (before the other
    # camera's first stamp, after its last) can never pair: whichever camera is
    # armed first catches an extra PTP boundary, and the recorder stops between
    # the two cameras' last frames. Report them, but only unmatched frames INSIDE
    # the common window count as a failure.
    lo, hi = max(L[0], R[0]) - tol, min(L[-1], R[-1]) + tol
    edge_l = sum(1 for t in L if t < lo or t > hi)
    edge_r = sum(1 for t in R if t < lo or t > hi)
    inner_l, inner_r = unl - edge_l, unr - edge_r
    print(f"  pairs: matched={matched} unmatched inside common window left={inner_l} right={inner_r} "
          f"(edge frames outside it: left={edge_l} right={edge_r}) "
          f"delta avg={dsum / matched / 1000 if matched else 0:.1f}µs max={dmax / 1000:.1f}µs")
    ok &= inner_l == 0 and inner_r == 0
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
