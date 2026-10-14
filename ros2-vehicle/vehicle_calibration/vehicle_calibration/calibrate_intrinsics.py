#!/usr/bin/env python3
"""Camera intrinsics calibration (live ROS topic, offline video, or rosbag)."""

import argparse
import json
import os
import shutil
import sqlite3
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image

# Fallback when the node is started without launch params (overridden by vehicle_params.yaml).
_DEFAULT_INTRINSICS_PATH = 'calibration/camera_intrinsics.json'
from vehicle_calibration.device_id import probe_v4l2_device_id

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ModuleNotFoundError:
    SequentialReader = None
    StorageOptions = None
    ConverterOptions = None

MIN_SAMPLES = 8
CORNER_SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


def make_object_points(rows: int, columns: int, square_size: float) -> np.ndarray:
    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2) * square_size
    return objp


def crop_zed_right_half(frame: np.ndarray) -> np.ndarray:
    height, width, _ = frame.shape
    return frame[:, width // 2 :]


def mean_corner_shift(current: np.ndarray, previous: np.ndarray | None) -> float | None:
    if previous is None:
        return None
    return float(
        np.linalg.norm(
            current.reshape(-1, 2) - previous.reshape(-1, 2),
            axis=1,
        ).mean()
    )


def solve_and_save_calibration(
    objpoints: list,
    imgpoints: list,
    image_size: tuple[int, int],
    output_path: str | Path,
    device_id: str = '',
    log=None,
) -> bool:
    if len(objpoints) < MIN_SAMPLES:
        message = f'Not enough valid chessboard detections to calibrate (need at least {MIN_SAMPLES}).'
        if log:
            log.error(message)
        else:
            print(message)
        return False

    width, height = image_size
    if log:
        log.info(
            f'Running cv2.calibrateCamera with {len(objpoints)} samples at '
            f'{width}x{height}'
        )
    ret, mtx, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )
    output_path = Path(output_path)
    document = {
        'distortion_model': 'plumb_bob',
        'camera_matrix': np.asarray(mtx, dtype=np.float64).reshape(3, 3).tolist(),
        'distortion_coefficients': np.asarray(dist, dtype=np.float64).reshape(-1).tolist(),
        'width': int(width),
        'height': int(height),
    }
    if device_id:
        document['device_id'] = device_id
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2)
        handle.write('\n')
    summary = (
        f'Saved calibration to {output_path}\n'
        f'Reprojection success: {ret}\n'
        f'Camera matrix:\n{mtx}\n'
        f'Distortion coefficients:\n{dist}'
    )
    if log:
        log.info(summary)
    else:
        print(summary)
    return True


class ChessboardSampleCollector:
    """Accumulate chessboard detections from BGR frames."""

    def __init__(
        self,
        rows: int,
        columns: int,
        square_size: float,
        min_corner_shift_px: float = 20.0,
        zed_right_half: bool = False,
    ):
        self.chessboard_size = (rows, columns)
        self.objp = make_object_points(rows, columns, square_size)
        self.min_corner_shift_px = min_corner_shift_px
        self.zed_right_half = zed_right_half
        self.objpoints: list = []
        self.imgpoints: list = []
        self.last_accepted_corners = None
        self.gray_size: tuple[int, int] | None = None
        self.processed_frames = 0
        self.detected_frames = 0

    def process_frame(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        """Returns (accepted, display_frame)."""
        self.processed_frames += 1
        if self.zed_right_half:
            frame = crop_zed_right_half(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.gray_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        if not found:
            return False, frame

        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            CORNER_SUBPIX_CRITERIA,
        )
        shift = mean_corner_shift(corners_refined, self.last_accepted_corners)
        if shift is not None and shift < self.min_corner_shift_px:
            return False, frame

        self.detected_frames += 1
        self.objpoints.append(self.objp.copy())
        self.imgpoints.append(corners_refined)
        self.last_accepted_corners = corners_refined.copy()
        cv2.drawChessboardCorners(frame, self.chessboard_size, corners_refined, True)
        return True, frame

    def finalize(
        self,
        output_path: str | Path,
        device_id: str = '',
        log=None,
    ) -> bool:
        if self.gray_size is None:
            return False
        return solve_and_save_calibration(
            self.objpoints,
            self.imgpoints,
            self.gray_size,
            output_path,
            device_id=device_id,
            log=log,
        )


class TopicCalibrator(Node):
    def __init__(self):
        super().__init__('topic_calibrator')

        self.declare_parameter('image_topic', '/zed/left_camera/image_raw')
        self.declare_parameter('rows', 8)
        self.declare_parameter('columns', 5)
        self.declare_parameter('square_size', 0.0255)
        self.declare_parameter('frame_skip', 20)
        self.declare_parameter('zed_right_half', False)
        self.declare_parameter('output_path', _DEFAULT_INTRINSICS_PATH)
        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('device_id', '')
        self.declare_parameter('show_preview', True)
        self.declare_parameter('max_samples', 30)
        self.declare_parameter('min_corner_shift_px', 20.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.device_path = str(self.get_parameter('device_path').value)
        configured_device_id = str(self.get_parameter('device_id').value).strip()
        self.device_id = configured_device_id or probe_v4l2_device_id(self.device_path)
        self.frame_skip = int(self.get_parameter('frame_skip').value)
        self.output_path = self.get_parameter('output_path').value
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.max_samples = int(self.get_parameter('max_samples').value)

        self.collector = ChessboardSampleCollector(
            rows=int(self.get_parameter('rows').value),
            columns=int(self.get_parameter('columns').value),
            square_size=float(self.get_parameter('square_size').value),
            min_corner_shift_px=float(self.get_parameter('min_corner_shift_px').value),
            zed_right_half=bool(self.get_parameter('zed_right_half').value),
        )

        self.last_image_time = self.get_clock().now()
        self.idle_timeout_sec = 5.0
        self.idle_timer = self.create_timer(0.5, self.check_idle_timeout)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved = False
        self.stop_requested = False

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Subscribed to {self.image_topic} for intrinsics calibration')

    def image_callback(self, msg):
        if self.stop_requested:
            return
        self.last_image_time = self.get_clock().now()
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        accepted, frame = self.collector.process_frame(frame)
        self._draw_status(frame, accepted=accepted)
        self._show_frame(frame)

        if self.collector.processed_frames % 20 == 0:
            self._log_detection_stats()

        if len(self.collector.objpoints) >= self.max_samples:
            self.get_logger().info(
                f'Reached {len(self.collector.objpoints)} diverse detections, stopping...'
            )
            self.stop_requested = True

    def check_idle_timeout(self):
        if self.stop_requested:
            return
        elapsed = self.get_clock().now() - self.last_image_time
        if elapsed.nanoseconds > int(self.idle_timeout_sec * 1e9):
            self.get_logger().info('No images received recently, stopping...')
            self.stop_requested = True

    def _show_frame(self, frame):
        if not self.show_preview:
            return
        cv2.imshow('calibration', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit requested, calibrating...')
            self.stop_requested = True

    def _draw_status(self, frame, accepted: bool):
        ratio = (
            self.collector.detected_frames / self.collector.processed_frames
            if self.collector.processed_frames
            else 0.0
        )
        color = (0, 255, 0) if accepted else (0, 0, 255)
        cv2.putText(
            frame,
            f'processed={self.collector.processed_frames} '
            f'detected={self.collector.detected_frames} ratio={ratio:.2%}',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    def _log_detection_stats(self):
        ratio = (
            self.collector.detected_frames / self.collector.processed_frames
            if self.collector.processed_frames
            else 0.0
        )
        self.get_logger().info(
            f'frames={self.frame_count} processed={self.collector.processed_frames} '
            f'detections={self.collector.detected_frames} ratio={ratio:.2%}'
        )

    def finalize(self):
        if self.saved:
            return
        cv2.destroyAllWindows()
        self.saved = self.collector.finalize(
            self.output_path,
            device_id=self.device_id,
            log=self.get_logger(),
        )


def run_ros_node(args=None):
    rclpy.init(args=args)
    node = TopicCalibrator()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finalize()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def calibrate_from_frame_iter(
    frames,
    collector: ChessboardSampleCollector,
    frame_skip: int,
    show_preview: bool,
    max_samples: int | None,
) -> None:
    frame_count = 0
    stop = False
    for frame in frames:
        frame_count += 1
        if frame_count % frame_skip != 0:
            continue
        accepted, display = collector.process_frame(frame)
        if show_preview:
            cv2.imshow('calibration', display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop = True
        if max_samples and len(collector.objpoints) >= max_samples:
            stop = True
        if stop:
            break
    cv2.destroyAllWindows()


def iter_video_frames(video_path: Path):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f'Could not open video: {video_path}')
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        yield frame
    cap.release()


def open_bag_reader(bag_path: str, storage_id: str):
    if SequentialReader is None:
        raise RuntimeError('rosbag2_py is not available')
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=storage_id),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    return reader


def find_sqlite_topic_shards(bag_path: str, topic_name: str):
    if not os.path.isdir(bag_path):
        raise RuntimeError(f'Bag path is not a directory: {bag_path}')
    db3_files = sorted(
        os.path.join(bag_path, name) for name in os.listdir(bag_path) if name.endswith('.db3')
    )
    if not db3_files:
        raise RuntimeError(f'No .db3 files found in bag directory: {bag_path}')

    available_topics: dict[str, str] = {}
    shards: list[tuple[str, int]] = []
    topic_type = None
    for db_path in db3_files:
        conn = sqlite3.connect(db_path)
        try:
            cur = conn.cursor()
            cur.execute('SELECT id, name, type FROM topics')
            for topic_id, name, type_name in cur.fetchall():
                available_topics[name] = type_name
                if name == topic_name:
                    topic_type = type_name
                    shards.append((db_path, topic_id))
        finally:
            conn.close()

    if not shards:
        available = ', '.join(sorted(available_topics)) or '<none>'
        raise RuntimeError(f'Topic not found: {topic_name}. Available topics: {available}')
    return topic_type, shards


def iter_sqlite_serialized_messages(shards):
    for db_path, topic_id in shards:
        conn = sqlite3.connect(db_path)
        try:
            cur = conn.cursor()
            cur.execute(
                'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp',
                (topic_id,),
            )
            for (data,) in cur:
                yield data
        finally:
            conn.close()


def iter_rosbag2_serialized_messages(bag_path: str, topic_name: str, storage_id: str):
    reader = open_bag_reader(bag_path, storage_id)
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        if topic == topic_name:
            yield data


def get_bag_message_stream(bag_path: str, topic_name: str, storage_id: str):
    if SequentialReader is not None:
        reader = open_bag_reader(bag_path, storage_id)
        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        if topic_name not in topic_types:
            available = ', '.join(sorted(topic_types)) or '<none>'
            raise RuntimeError(f'Topic not found: {topic_name}. Available topics: {available}')
        return topic_types[topic_name], iter_rosbag2_serialized_messages(
            bag_path, topic_name, storage_id
        )

    if storage_id != 'sqlite3':
        raise RuntimeError(
            f"rosbag2_py is not available; use --storage sqlite3 for .db3 bags (got '{storage_id}')."
        )
    topic_type, shards = find_sqlite_topic_shards(bag_path, topic_name)
    return topic_type, iter_sqlite_serialized_messages(shards)


def iter_bag_frames(bag_path: str, topic_name: str, storage_id: str, max_frames: int | None = None):
    topic_type, serialized_messages = get_bag_message_stream(bag_path, topic_name, storage_id)
    msg_type = get_message(topic_type)
    bridge = CvBridge()
    count = 0
    for data in serialized_messages:
        msg = deserialize_message(data, msg_type)
        yield bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        count += 1
        if max_frames is not None and count >= max_frames:
            break


def cmd_from_video(args):
    collector = ChessboardSampleCollector(
        rows=args.rows,
        columns=args.columns,
        square_size=args.square_size,
        min_corner_shift_px=args.min_corner_shift_px,
        zed_right_half=args.zed_right_half,
    )
    calibrate_from_frame_iter(
        iter_video_frames(Path(args.video)),
        collector,
        frame_skip=args.frame_skip,
        show_preview=args.preview,
        max_samples=args.max_samples,
    )
    device_id = probe_v4l2_device_id(args.device_path)
    if not collector.finalize(args.output, device_id=device_id):
        return 1
    return 0


def cmd_from_bag(args):
    collector = ChessboardSampleCollector(
        rows=args.rows,
        columns=args.columns,
        square_size=args.square_size,
        min_corner_shift_px=args.min_corner_shift_px,
        zed_right_half=args.zed_right_half,
    )
    frames = iter_bag_frames(args.bag, args.topic, args.storage, args.max_frames)
    calibrate_from_frame_iter(
        frames,
        collector,
        frame_skip=args.frame_skip,
        show_preview=args.preview,
        max_samples=args.max_samples,
    )
    if not collector.finalize(args.output):
        return 1
    return 0


def cmd_bag_to_video(args):
    topic_type, serialized_messages = get_bag_message_stream(args.bag, args.topic, args.storage)
    msg_type = get_message(topic_type)
    bridge = CvBridge()
    writer = None
    frames = 0
    for data in serialized_messages:
        msg = deserialize_message(data, msg_type)
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(args.output, fourcc, args.fps, (width, height))
            if not writer.isOpened():
                raise RuntimeError(f'Could not open video writer for {args.output}')
        writer.write(frame)
        frames += 1
        if args.max_frames is not None and frames >= args.max_frames:
            break
    if writer:
        writer.release()
    print(f'Wrote {frames} frames to {args.output}')
    return 0


def read_video_metadata(video_path: Path):
    cap = cv2.VideoCapture(str(video_path))
    if cap.isOpened():
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        return width, height, frame_count, 'OpenCV'

    ffprobe = shutil.which('ffprobe')
    if ffprobe is None:
        raise RuntimeError('Could not read video metadata (OpenCV failed and ffprobe missing)')

    result = subprocess.run(
        [
            ffprobe,
            '-v',
            'error',
            '-select_streams',
            'v:0',
            '-show_entries',
            'stream=width,height,nb_frames',
            '-of',
            'json',
            str(video_path),
        ],
        capture_output=True,
        check=True,
        text=True,
    )
    data = json.loads(result.stdout)
    streams = data.get('streams', [])
    if not streams:
        raise RuntimeError(f'No video stream found in {video_path}')
    stream = streams[0]
    width = int(stream['width'])
    height = int(stream['height'])
    frame_count = stream.get('nb_frames')
    frame_count = int(frame_count) if frame_count and str(frame_count).isdigit() else frame_count
    return width, height, frame_count, 'ffprobe'


def cmd_video_info(args):
    video_path = Path(args.video)
    if not video_path.exists():
        print(f'Video file not found: {video_path}')
        return 1
    width, height, frame_count, source = read_video_metadata(video_path)
    print(f'Video: {video_path}')
    print(f'Width: {width}')
    print(f'Height: {height}')
    print(f'Resolution: {width}x{height}')
    if frame_count is not None:
        print(f'Frame count: {frame_count}')
    print(f'Read using: {source}')
    return 0


def cmd_record_video(args):
    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output, fourcc, args.fps, (args.width, args.height))
    print('Recording... Press Ctrl+C to stop or q in the preview window.')
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            out.write(frame)
            if args.preview:
                try:
                    cv2.imshow('Recording...', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except Exception:
                    pass
    except KeyboardInterrupt:
        print('Recording stopped by user.')
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print(f'Video saved as {args.output} ({args.width}x{args.height} @ {args.fps} FPS).')
    return 0


def add_chessboard_args(parser, defaults: dict):
    parser.add_argument('--rows', type=int, default=defaults['rows'])
    parser.add_argument('--columns', type=int, default=defaults['columns'])
    parser.add_argument('--square-size', type=float, default=defaults['square_size'])
    parser.add_argument('--frame-skip', type=int, default=defaults['frame_skip'])
    parser.add_argument('--min-corner-shift-px', type=float, default=defaults['min_corner_shift_px'])
    parser.add_argument('--zed-right-half', action='store_true', default=defaults['zed_right_half'])
    parser.add_argument('--preview', action='store_true', default=defaults['preview'])
    parser.add_argument('--max-samples', type=int, default=defaults['max_samples'])
    parser.add_argument(
        '--output',
        default=defaults['output'],
        help='Output JSON path',
    )


def build_cli_parser() -> argparse.ArgumentParser:
    defaults = {
        'rows': 8,
        'columns': 5,
        'square_size': 0.0255,
        'frame_skip': 10,
        'min_corner_shift_px': 20.0,
        'zed_right_half': False,
        'preview': True,
        'max_samples': None,
        'output': _DEFAULT_INTRINSICS_PATH,
    }

    parser = argparse.ArgumentParser(
        description='Camera intrinsics calibration (ROS topic, video file, or rosbag).',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    run_parser = sub.add_parser('run', help='Live calibration from a ROS image topic.')
    run_parser.set_defaults(func=lambda _a: run_ros_node())

    video_parser = sub.add_parser('from-video', help='Calibrate from a video file.')
    video_parser.add_argument('--video', required=True, help='Path to input video.')
    video_parser.add_argument('--device-path', default='/dev/video0', help='Camera device for device_id metadata.')
    add_chessboard_args(video_parser, defaults)
    video_parser.set_defaults(func=cmd_from_video)

    bag_parser = sub.add_parser('from-bag', help='Calibrate directly from a rosbag image topic.')
    bag_parser.add_argument('bag', help='Bag directory or file.')
    bag_parser.add_argument('--topic', default='/zed/left_camera/image_raw')
    bag_parser.add_argument('--storage', default='sqlite3', help='rosbag2 storage id (e.g. mcap).')
    bag_parser.add_argument('--max-frames', type=int, default=None)
    add_chessboard_args(bag_parser, {**defaults, 'frame_skip': 20})
    bag_parser.set_defaults(func=cmd_from_bag)

    bag_video_parser = sub.add_parser('bag-to-video', help='Export a bag image topic to MP4.')
    bag_video_parser.add_argument('bag')
    bag_video_parser.add_argument('--topic', default='/zed/left_camera/image_raw')
    bag_video_parser.add_argument('--output', default='zed_left.mp4')
    bag_video_parser.add_argument('--fps', type=float, default=30.0)
    bag_video_parser.add_argument('--storage', default='sqlite3')
    bag_video_parser.add_argument('--max-frames', type=int, default=None)
    bag_video_parser.set_defaults(func=cmd_bag_to_video)

    info_parser = sub.add_parser('video-info', help='Print resolution and frame count for a video file.')
    info_parser.add_argument('--video', required=True)
    info_parser.set_defaults(func=cmd_video_info)

    record_parser = sub.add_parser('record-video', help='Record chessboard footage from a V4L2 device to MP4.')
    record_parser.add_argument('--device', type=int, default=0, help='OpenCV device index (Elgato is usually 0).')
    record_parser.add_argument('--output', default='video_calibracao2.mp4')
    record_parser.add_argument('--width', type=int, default=1280)
    record_parser.add_argument('--height', type=int, default=720)
    record_parser.add_argument('--fps', type=float, default=30.0)
    record_parser.add_argument('--preview', action='store_true', default=True)
    record_parser.set_defaults(func=cmd_record_video)

    return parser


def main_cli(argv=None):
    parser = build_cli_parser()
    args = parser.parse_args(argv)
    result = args.func(args)
    return 0 if result is None else int(result)


# Backward-compatible entry points (deprecated; use calibrate_intrinsics subcommands).
def legacy_calibrate_camera():
    return main_cli([
        'from-video',
        '--video',
        './video_calibracao2.mp4',
        '--output',
        _DEFAULT_INTRINSICS_PATH,
    ])


def legacy_camera_to_video():
    return main_cli(['bag-to-video', './bag'])


def legacy_get_video_frame_size():
    return main_cli(['video-info', '--video', 'calibration_elgato_video.mp4'])


def legacy_record_intrinsics_video():
    return main_cli(['record-video'])


def main():
    """Default: live ROS node when run via ros2 run without subcommands; otherwise CLI."""
    if len(sys.argv) <= 1:
        run_ros_node()
        return
    raise SystemExit(main_cli())


if __name__ == '__main__':
    main()
