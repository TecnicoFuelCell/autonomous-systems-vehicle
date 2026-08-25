#!/usr/bin/env python3

# Script that tries to find the best odom fit to the gps locations captured over a trajectory, 
# and prints the rpm_ratio, steering_gain and steering_offse that results in the lowest error (rmse)

# Requires being passed a path to a bag where /dir_data, /vesc_data, /gps_pose are published

import argparse
import math
import sqlite3
import statistics
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def resolve_db3(path):
    path = Path(path)
    if path.is_dir():
        db3_files = sorted(path.glob("*.db3"))
        if not db3_files:
            raise FileNotFoundError(f"No .db3 file found in {path}")
        return db3_files[0]
    return path


def load_topic(con, topics, topic_name):
    topic_id, msg_type = topics[topic_name]
    rows = []
    for record_time_ns, data in con.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp",
        (topic_id,),
    ):
        msg = deserialize_message(data, msg_type)
        if hasattr(msg, "header"):
            header_time = stamp_to_sec(msg.header.stamp)
            t = header_time if header_time > 0.0 else record_time_ns * 1e-9
        else:
            t = record_time_ns * 1e-9
        rows.append((t, msg))
    return rows


def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def fit_rigid(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = target_centered.T @ source_centered / len(source)
    u, _singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(2)
    if np.linalg.det(u @ vt) < 0.0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    translation = target_mean - rotation @ source_mean
    aligned = (rotation @ source.T).T + translation
    rmse = math.sqrt(float(np.mean(np.sum((aligned - target) ** 2, axis=1))))
    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return rmse, yaw_deg


def nearest_pairs(sim_times, sim_points, gps_times, gps_points):
    paired_sim = []
    paired_gps = []
    dt = []
    for gps_t, gps_xy in zip(gps_times, gps_points):
        index = int(np.argmin(np.abs(sim_times - gps_t)))
        paired_sim.append(sim_points[index])
        paired_gps.append(gps_xy)
        dt.append(float(sim_times[index] - gps_t))
    return np.asarray(paired_sim), np.asarray(paired_gps), dt


def simulate(vesc, steering, rpm_ratio, steering_gain, steering_offset):
    x = 0.0
    y = 0.0
    yaw = 0.0
    last_time = None
    steering_index = 0
    current_steering = 0.0
    times = []
    points = []

    for t, rpm in vesc:
        while steering_index < len(steering) and steering[steering_index][0] <= t:
            current_steering = steering[steering_index][1]
            steering_index += 1

        real_rpm = rpm * 0.1
        velocity = real_rpm * 2.0 * math.pi / 60.0 * 0.2825 * rpm_ratio

        if last_time is None:
            last_time = t
            times.append(t)
            points.append((x, y))
            continue

        dt = t - last_time
        last_time = t
        if dt <= 0.0 or dt > 1.0:
            times.append(t)
            points.append((x, y))
            continue

        steering_command = current_steering - steering_offset
        distance = velocity * dt

        if abs(steering_command) < 1e-9:
            body_dx = distance
            body_dy = 0.0
            body_dyaw = 0.0
        else:
            radius = steering_gain / steering_command
            body_dyaw = distance / radius
            body_dx = radius * math.sin(body_dyaw)
            body_dy = radius * (1.0 - math.cos(body_dyaw))

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x += cos_yaw * body_dx - sin_yaw * body_dy
        y += sin_yaw * body_dx + cos_yaw * body_dy
        yaw += body_dyaw

        times.append(t)
        points.append((x, y))

    return np.asarray(times), np.asarray(points), yaw


def score_model(vesc, steering, gps_times, gps_points, rpm_ratio, steering_gain, steering_offset):
    sim_times, sim_points, yaw = simulate(
        vesc,
        steering,
        rpm_ratio,
        steering_gain,
        steering_offset,
    )
    paired_sim, paired_gps, _dt = nearest_pairs(sim_times, sim_points, gps_times, gps_points)
    rmse, align_yaw_deg = fit_rigid(paired_sim, paired_gps)
    return {
        "rmse": rmse,
        "align_yaw_deg": align_yaw_deg,
        "length": path_length(sim_points),
        "yaw_change_deg": math.degrees(yaw),
        "rpm_ratio": rpm_ratio,
        "steering_gain": steering_gain,
        "steering_offset": steering_offset,
    }


def sweep(vesc, steering, gps_times, gps_points, rpm_ratios, gains, offsets):
    best = None
    for rpm_ratio in rpm_ratios:
        for steering_gain in gains:
            if abs(steering_gain) < 1e-9:
                continue
            for steering_offset in offsets:
                result = score_model(
                    vesc,
                    steering,
                    gps_times,
                    gps_points,
                    rpm_ratio,
                    steering_gain,
                    steering_offset,
                )
                if not math.isfinite(result["rmse"]):
                    continue
                if best is None or result["rmse"] < best["rmse"]:
                    best = result
    return best


def make_range(center, span, count):
    if count <= 1:
        return np.asarray([center])
    return np.linspace(center - span, center + span, count)


def main():
    parser = argparse.ArgumentParser(
        description="Fit wheel odom RPM and steering-radius parameters against /gps_pose."
    )
    parser.add_argument("bag", help="ROS 2 bag directory or .db3 file")
    parser.add_argument("--gps-topic", default="/gps_pose")
    parser.add_argument("--vesc-topic", default="/vesc_data")
    parser.add_argument("--dir-topic", default="/dir_data")
    parser.add_argument("--rpm-ratio-min", type=float, default=0.15)
    parser.add_argument("--rpm-ratio-max", type=float, default=0.35)
    parser.add_argument("--gain-min", type=float, default=-30000.0)
    parser.add_argument("--gain-max", type=float, default=30000.0)
    parser.add_argument("--offset-min", type=float, default=-20.0)
    parser.add_argument("--offset-max", type=float, default=20.0)
    args = parser.parse_args()

    db3 = resolve_db3(args.bag)
    con = sqlite3.connect(str(db3))
    topics = {
        name: (topic_id, get_message(msg_type))
        for topic_id, name, msg_type in con.execute("select id, name, type from topics")
    }
    for topic in (args.gps_topic, args.vesc_topic, args.dir_topic):
        if topic not in topics:
            raise RuntimeError(f"Missing required topic {topic}")

    gps_rows = load_topic(con, topics, args.gps_topic)
    vesc_rows = load_topic(con, topics, args.vesc_topic)
    steering_rows = load_topic(con, topics, args.dir_topic)

    gps_times = np.asarray([t for t, _msg in gps_rows])
    gps_points = np.asarray(
        [(msg.pose.position.x, msg.pose.position.y) for _t, msg in gps_rows],
        dtype=float,
    )
    vesc = [(t, float(msg.rpm)) for t, msg in vesc_rows]
    steering = [(t, float(msg.dir)) for t, msg in steering_rows]

    gps_len = path_length(gps_points)
    gps_duration = gps_times[-1] - gps_times[0]
    print(f"bag: {db3}")
    print(f"gps samples={len(gps_points)} duration={gps_duration:.2f}s length={gps_len:.2f}m")
    print(f"vesc samples={len(vesc)} dir samples={len(steering)}")
    print(f"dir median={statistics.median(v for _t, v in steering):.3f}")

    coarse = sweep(
        vesc,
        steering,
        gps_times,
        gps_points,
        np.linspace(args.rpm_ratio_min, args.rpm_ratio_max, 9),
        np.concatenate(
            [
                np.linspace(args.gain_min, -1000.0, 30),
                np.linspace(1000.0, args.gain_max, 30),
            ]
        ),
        np.linspace(args.offset_min, args.offset_max, 9),
    )

    best = coarse
    for gain_span, offset_span, rpm_span in ((4000.0, 8.0, 0.06), (1000.0, 2.0, 0.015)):
        best = sweep(
            vesc,
            steering,
            gps_times,
            gps_points,
            make_range(best["rpm_ratio"], rpm_span, 9),
            make_range(best["steering_gain"], gain_span, 17),
            make_range(best["steering_offset"], offset_span, 9),
        )

    print("\nbest fit:")
    print(f"  rmse:             {best['rmse']:.2f} m")
    print(f"  align_yaw:        {best['align_yaw_deg']:.1f} deg")
    print(f"  simulated length: {best['length']:.2f} m")
    print(f"  yaw change:       {best['yaw_change_deg']:.1f} deg")
    print(f"  rpm_ratio:        {best['rpm_ratio']:.6f}")
    print(f"  steering_gain:    {best['steering_gain']:.6f}")
    print(f"  steering_offset:  {best['steering_offset']:.6f}")

    print("\nuse this model:")
    print("  steering_command = dir - steering_offset")
    print("  turn_radius = steering_gain / steering_command")
    print("  omega_z = linear_velocity / turn_radius")


if __name__ == "__main__":
    main()
