#!/usr/bin/env python3
import argparse
import sys
import time

try:
    import cv2
except Exception as e:
    cv2 = None

try:
    from PIL import Image
except Exception:
    Image = None

import rospy
from sensor_msgs.msg import CompressedImage

def capture_frame(device_index: int, width: int = None, height: int = None):
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is not available. Please install python3-opencv.")
    cap = cv2.VideoCapture(device_index)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video device index {device_index}")
    # Warm up a bit
    for _ in range(3):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError("Failed to capture frame from camera")
    return frame


def encode_jpeg_under_limit(frame, max_bytes: int, initial_quality: int = 85):
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is not available.")

    def try_encode(img, quality):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        ok, buf = cv2.imencode('.jpg', img, params)
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        b = buf.tobytes()
        return b

    # First pass: reduce quality
    for quality in [initial_quality, 80, 70, 60, 50, 40, 30]:
        data = try_encode(frame, quality)
        if len(data) <= max_bytes:
            return data, quality, frame.shape[1], frame.shape[0]

    # Second pass: downscale by half and try qualities again
    h, w = frame.shape[:2]
    small = cv2.resize(frame, (max(1, w//2), max(1, h//2)))
    for quality in [85, 80, 70, 60, 50, 40, 30]:
        data = try_encode(small, quality)
        if len(data) <= max_bytes:
            return data, quality, small.shape[1], small.shape[0]

    # As a last resort, return the smallest we got (may exceed limit)
    data = try_encode(small, 30)
    return data, 30, small.shape[1], small.shape[0]


def reencode_file_jpeg_under_limit(path: str, max_bytes: int):
    if Image is None:
        # PIL not available; return original bytes
        with open(path, 'rb') as f:
            data = f.read()
        return data, None, 0, 0

    img = Image.open(path)
    img = img.convert('RGB')  # ensure JPEG-compatible

    import io
    # Try qualities
    for quality in [85, 80, 70, 60, 50, 40, 30]:
        bio = io.BytesIO()
        img.save(bio, format='JPEG', quality=quality, optimize=True)
        data = bio.getvalue()
        if len(data) <= max_bytes:
            return data, quality, img.size[0], img.size[1]

    # Downscale and try again
    w, h = img.size
    img2 = img.resize((max(1, w//2), max(1, h//2)))
    for quality in [85, 80, 70, 60, 50, 40, 30]:
        bio = io.BytesIO()
        img2.save(bio, format='JPEG', quality=quality, optimize=True)
        data = bio.getvalue()
        if len(data) <= max_bytes:
            return data, quality, img2.size[0], img2.size[1]

    # Last resort
    bio = io.BytesIO()
    img2.save(bio, format='JPEG', quality=30, optimize=True)
    data = bio.getvalue()
    return data, 30, img2.size[0], img2.size[1]


def main():
    parser = argparse.ArgumentParser(description="Capture one camera frame and publish as sensor_msgs/CompressedImage")
    parser.add_argument('--device', type=int, default=0, help='Camera device index (default: 0)')
    parser.add_argument('--topic', type=str, default='/fbrtk/jpeg', help='ROS topic to publish (default: /fbrtk/jpeg)')
    parser.add_argument('--width', type=int, default=None, help='Capture width')
    parser.add_argument('--height', type=int, default=None, help='Capture height')
    parser.add_argument('--max_kb', type=int, default=None, help='Max JPEG size KB (overrides param)')
    parser.add_argument('--file', type=str, default=None, help='Publish this JPEG file instead of capturing from camera')
    args = parser.parse_args()

    rospy.init_node('capture_and_publish_jpeg', anonymous=True)
    pub = rospy.Publisher(args.topic, CompressedImage, queue_size=1)

    # Determine size limit: prefer param from running driver, fallback to flag or 100KB
    max_kb = rospy.get_param('/fbrtk_driver/jpeg_max_size_kb', None)
    if max_kb is None:
        max_kb = args.max_kb if args.max_kb is not None else 100
    max_bytes = int(max_kb) * 1024

    if args.file:
        # Load and (if possible) re-encode from file under limit
        try:
            data, quality, w, h = reencode_file_jpeg_under_limit(args.file, max_bytes)
        except Exception as e:
            print(f"[ERROR] Failed to (re)encode file {args.file}: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        try:
            frame = capture_frame(args.device, args.width, args.height)
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            sys.exit(1)

        try:
            data, quality, w, h = encode_jpeg_under_limit(frame, max_bytes)
        except Exception as e:
            print(f"[ERROR] JPEG encode failed: {e}", file=sys.stderr)
            sys.exit(1)

    if len(data) > max_bytes:
        print(f"[WARN] Encoded JPEG {len(data)} bytes still exceeds limit {max_bytes} bytes; publishing anyway", file=sys.stderr)

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'jpeg'
    msg.data = bytearray(data)

    # small delay to ensure connection
    time.sleep(0.3)
    pub.publish(msg)
    if args.file:
        print(f"Published {len(data)} bytes from file {args.file} as JPEG (quality={quality}, size={w}x{h}) to {args.topic}")
    else:
        print(f"Published {len(data)} bytes as JPEG (quality={quality}, size={w}x{h}) to {args.topic}")

if __name__ == '__main__':
    main()
