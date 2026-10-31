#!/usr/bin/env python3
"""Virtual camera: stream an image or video file to a v4l2loopback device.

Test fixture that lets webcam_node (and everything downstream of it) run when
the physical camera is not connected. It reads the same camera parameters as
webcam_node from vehicle_params.yaml (camera_device_path, frame_width,
frame_height, camera_fps, pixel_format), so it produces exactly the mode
webcam_node asks for. Only the input file and playback options are given on
the command line.

Python decodes/resizes/pads frames with OpenCV and pipes raw BGR bytes into
GStreamer, which converts them and writes to the device through v4l2sink:

    gst-launch-1.0 fdsrc fd=0 ! rawvideoparse format=bgr ... ! videoconvert
        ! (YUY2 caps | jpegenc for MJPG) ! v4l2sink device=<camera_device_path>

GStreamer is used because cv2.VideoWriter cannot open a v4l2loopback device
and raw write() to it fails with EINVAL on the tested build.

This node does not create the device (loading a kernel module needs root on
the host). Create it on the host before starting the container, with the real
camera unplugged and video_nr matching camera_device_path:

    sudo modprobe v4l2loopback video_nr=0 card_label="TFC virtual camera" exclusive_caps=1
    sudo modprobe -r v4l2loopback    # remove it when done

See docs/virtual-devices.md in the repository for details. If the
real camera owns camera_device_path, v4l2sink fails and this node exits with
an error instead of interfering with it.

Examples:
    ros2 run virtual_devices virtual_camera_node clip.mp4 \
        --ros-args --params-file <vehicle_params.yaml>
    ros2 run virtual_devices virtual_camera_node frame.png --frames 90 \
        --ros-args --params-file <vehicle_params.yaml>
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff'}
GST_LAUNCH = 'gst-launch-1.0'
SUPPORTED_PIXEL_FORMATS = ('MJPG', 'YUYV')


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog='virtual_camera_node',
        description='Stream an image or video file to the v4l2loopback device at '
                    'camera_device_path, in the mode configured for webcam_node.',
    )
    p.add_argument('input', help='Path to input image or video file.')
    loop = p.add_mutually_exclusive_group()
    loop.add_argument('--loop', dest='loop', action='store_true',
                      help='Loop video / repeat image forever (default).')
    loop.add_argument('--no-loop', dest='loop', action='store_false',
                      help='Play video once then exit.')
    p.set_defaults(loop=True)
    p.add_argument('--frames', type=int, default=0,
                   help='Stop after N frames (0 = infinite, default: 0).')
    p.add_argument('--timeout', type=float, default=0.0,
                   help='Stop after N seconds (0 = infinite, default: 0).')
    p.add_argument('--background', default='0,0,0',
                   help="Padding color as B,G,R (default: '0,0,0' = black).")
    p.add_argument('--stretch', action='store_true',
                   help='Stretch to fill output instead of preserving aspect '
                        'ratio with padding.')
    return p.parse_args(argv)


def parse_background(s):
    try:
        parts = [int(x) for x in s.split(',')]
        if len(parts) != 3 or not all(0 <= v <= 255 for v in parts):
            raise ValueError
        return tuple(parts)  # B, G, R
    except ValueError:
        raise SystemExit(f"Invalid --background {s!r}: expected 'B,G,R' 0-255.")


def resize_and_pad(frame_bgr, width, height, background, stretch):
    """Resize BGR frame to exactly (width, height)."""
    fh, fw = frame_bgr.shape[:2]
    if fw == width and fh == height:
        return frame_bgr
    if stretch:
        interp = cv2.INTER_LINEAR if fw * fh < width * height else cv2.INTER_AREA
        return cv2.resize(frame_bgr, (width, height), interpolation=interp)
    scale = min(width / fw, height / fh)
    nw, nh = max(1, int(fw * scale)), max(1, int(fh * scale))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame_bgr, (nw, nh), interpolation=interp)
    canvas = np.full((height, width, 3), background, dtype=np.uint8)
    y0 = (height - nh) // 2
    x0 = (width - nw) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas


def is_image_path(path):
    return os.path.splitext(path)[1].lower() in IMAGE_EXTS


def frame_generator(path, width, height, background, stretch, loop):
    """Yield raw BGR frame bytes sized exactly width x height."""
    if is_image_path(path):
        frame = cv2.imread(path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f'Cannot read image {path!r} with cv2.imread.')
        payload = np.ascontiguousarray(
            resize_and_pad(frame, width, height, background, stretch)).tobytes()
        while True:
            yield payload
        return

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open video {path!r} with cv2.VideoCapture.')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                if loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                return
            fitted = resize_and_pad(frame, width, height, background, stretch)
            yield np.ascontiguousarray(fitted).tobytes()
    finally:
        cap.release()


def build_gst_cmd(device, width, height, fps, pixel_format):
    # GStreamer caps need an integer N/1 framerate.
    fps_i = max(1, int(round(fps)))
    size = f'width={width},height={height},framerate={fps_i}/1'
    cmd = [
        GST_LAUNCH,
        'fdsrc', 'fd=0',
        '!', 'rawvideoparse', 'format=bgr', f'width={width}', f'height={height}',
        f'framerate={fps_i}/1',
        '!', 'videoconvert',
    ]
    if pixel_format == 'MJPG':
        cmd += ['!', 'video/x-raw,format=I420', '!', 'jpegenc', '!', f'image/jpeg,{size}']
    else:  # YUYV is called YUY2 in GStreamer
        cmd += ['!', f'video/x-raw,format=YUY2,{size}']
    return cmd + ['!', 'v4l2sink', f'device={device}']


class VirtualCamera(Node):
    def __init__(self, args):
        super().__init__('virtual_camera')

        # Same keys and defaults as webcam_node; values come from the shared /**
        # section of vehicle_bringup/config/vehicle_params.yaml.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_device_path', '/dev/video0'),
                ('frame_width', 1280),
                ('frame_height', 720),
                ('camera_fps', 60.0),
                ('pixel_format', 'MJPG'),
            ]
        )
        device = str(self.get_parameter('camera_device_path').value)
        width = int(self.get_parameter('frame_width').value)
        height = int(self.get_parameter('frame_height').value)
        fps = float(self.get_parameter('camera_fps').value)
        pixel_format = str(self.get_parameter('pixel_format').value).upper()

        if width <= 0 or height <= 0 or fps <= 0:
            raise RuntimeError('frame_width, frame_height and camera_fps must be positive.')
        if pixel_format not in SUPPORTED_PIXEL_FORMATS:
            raise RuntimeError(
                f'pixel_format {pixel_format!r} is not supported; '
                f'expected one of {SUPPORTED_PIXEL_FORMATS}.')
        if not os.path.isfile(args.input):
            raise RuntimeError(f'Input file not found: {args.input!r}')
        if not os.path.exists(device):
            nr = device.removeprefix('/dev/video')
            raise RuntimeError(
                f'{device} does not exist. Create it on the host (outside Docker) with:\n'
                f'  sudo modprobe v4l2loopback video_nr={nr} '
                f'card_label="TFC virtual camera" exclusive_caps=1\n'
                f'then pass it into the container. Remove it afterwards with:\n'
                f'  sudo modprobe -r v4l2loopback')
        if shutil.which(GST_LAUNCH) is None:
            raise RuntimeError(
                f'{GST_LAUNCH} not found. Install gstreamer1.0-tools, '
                f'gstreamer1.0-plugins-base and gstreamer1.0-plugins-good.')

        self.device = device
        self.max_frames = args.frames
        self.timeout = args.timeout
        self.count = 0
        self.done = False
        self.failed = False

        # Fetch the first frame now so unreadable input fails before GStreamer starts.
        self.frames = frame_generator(
            args.input, width, height, parse_background(args.background),
            args.stretch, args.loop)
        self.next_payload = next(self.frames, None)
        if self.next_payload is None:
            raise RuntimeError(f'No frames could be read from {args.input!r}')

        cmd = build_gst_cmd(device, width, height, fps, pixel_format)
        self.get_logger().info(
            f'Input:    {args.input} '
            f"({'image-repeat' if is_image_path(args.input) else 'video'}, "
            f'loop={args.loop}, stretch={args.stretch})')
        self.get_logger().info(f'Device:   {device} ({width}x{height} {pixel_format} @ {fps} fps)')
        self.get_logger().info(f"Pipeline: {' '.join(cmd)}")

        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.start = time.monotonic()
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        if self.done:
            return
        if self.max_frames and self.count >= self.max_frames:
            return self.finish(f'Reached --frames {self.max_frames}.')
        if self.timeout and time.monotonic() - self.start >= self.timeout:
            return self.finish(f'Reached --timeout {self.timeout} s.')
        if self.next_payload is None:
            return self.finish('End of video.')
        if self.proc.poll() is not None:
            return self.finish(
                f'GStreamer exited early (code {self.proc.returncode}) after '
                f'{self.count} frames.', failed=True)
        try:
            self.proc.stdin.write(self.next_payload)
        except BrokenPipeError:
            return self.finish(f'GStreamer pipe broke after {self.count} frames.', failed=True)
        self.count += 1
        if self.count % 300 == 0:
            self.get_logger().info(f'Wrote {self.count} frames')
        self.next_payload = next(self.frames, None)

    def finish(self, reason, failed=False):
        self.done = True
        self.failed = failed
        if failed:
            self.get_logger().error(reason)
        else:
            self.get_logger().info(reason)

    def close(self):
        """Stop GStreamer and report its errors if it failed."""
        self.timer.cancel()
        try:
            self.proc.stdin.close()
        except BrokenPipeError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            self.proc.wait()
        err_tail = self.proc.stderr.read().decode(errors='replace')[-2000:].strip()
        if self.failed and err_tail:
            self.get_logger().error(f'GStreamer stderr:\n{err_tail}')
        elapsed = time.monotonic() - self.start
        if self.count > 0 and elapsed > 0:
            self.get_logger().info(
                f'Wrote {self.count} frames in {elapsed:.1f} s '
                f'({self.count / elapsed:.1f} fps) to {self.device}.')


def main(args=None):
    argv = sys.argv if args is None else args
    cli_args = parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv)

    node = None
    exit_code = 0
    try:
        node = VirtualCamera(cli_args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 1 if node.failed else 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        print(f'Error: {e}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
