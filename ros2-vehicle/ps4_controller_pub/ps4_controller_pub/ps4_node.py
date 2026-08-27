#!/usr/bin/env python3
"""
PS4 DualShock 4 reader for ROS 2.

Subscribes to /joy (sensor_msgs/Joy from joy_node) and publishes
car_msgs/msg/ImSpeed on /joystick_movement, compatible with
deadman_vesc_direction_imu_mag_pub (deploy_joystick).

Ubuntu 24.04 + hid-playstation + Docker notes:
- joy_node must have /dev/input passthrough (--device=/dev/input --group-add input)
- DS4 white bar = not paired; press PS until solid blue, then /dev/input/js0 appears
- Trigger polarity varies (idle +1 vs -1) -> handled via trigger_idle_value param and auto-inversion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from car_msgs.msg import ImSpeed


def map_value(value, from_min, from_max, to_min, to_max):
    if from_max == from_min:
        return to_min
    return int((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min)


class PS4Reader(Node):
    def __init__(self):
        super().__init__('ps4_reader')

        self.declare_parameters('', [
            ('joy_topic', '/joy'),
            ('output_topic', '/joystick_movement'),
            ('l2_axis', 2),
            ('r2_axis', 5),
            ('steer_axis', 0),
            ('deadzone', 0.05),
            ('trigger_idle', 1.0),  # idle value reported by driver (+1 or -1); 1.0 matches joystick_reader.cpp
            ('invert_steer', True),  # matches -axes[0] in C++ version
            ('use_buttons_fallback', True),  # if axes too short, try buttons[6]/[7] for L2/R2
            ('debug_interval', 100),  # log every N msgs
        ])

        self.joy_topic = self.get_parameter('joy_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.l2_axis = int(self.get_parameter('l2_axis').value)
        self.r2_axis = int(self.get_parameter('r2_axis').value)
        self.steer_axis = int(self.get_parameter('steer_axis').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.trigger_idle = float(self.get_parameter('trigger_idle').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.use_buttons_fallback = bool(self.get_parameter('use_buttons_fallback').value)
        self.debug_interval = int(self.get_parameter('debug_interval').value)

        self.get_logger().info(
            f'PS4Reader params:\n'
            f'  joy_topic: {self.joy_topic}\n'
            f'  output_topic: {self.output_topic}\n'
            f'  l2_axis: {self.l2_axis}, r2_axis: {self.r2_axis}, steer_axis: {self.steer_axis}\n'
            f'  deadzone: {self.deadzone}, trigger_idle: {self.trigger_idle}\n'
            f'  invert_steer: {self.invert_steer}, use_buttons_fallback: {self.use_buttons_fallback}'
        )
        self.get_logger().info('Waiting for PS4 /joy — ensure joy_node dev=/dev/input/js0 is blue (press PS)',)
        self.get_logger().info('Docker: run with --device=/dev/input --group-add $(getent group input | cut -d: -f3)')

        self.pub = self.create_publisher(ImSpeed, self.output_topic, 10)
        self.sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        self._count = 0
        self._first_log = True

    def _normalize_trigger(self, raw):
        """Convert raw axis (-1..1) to 0..1 trigger press amount handling both polarities."""
        # Original C++: (1 - raw)/2 assumes idle +1 -> 0, pressed -1 ->1
        # For idle -1 -> would be inverted, so detect and flip if trigger_idle==-1
        if self.trigger_idle > 0:
            norm = (1.0 - raw) / 2.0
        else:
            norm = (raw + 1.0) / 2.0
        # clamp
        if norm < 0.0:
            norm = 0.0
        if norm > 1.0:
            norm = 1.0
        # deadzone
        if norm < self.deadzone:
            norm = 0.0
        return norm

    def joy_callback(self, msg: Joy):
        # First message diagnostics
        if self._first_log:
            self.get_logger().info(f'First /joy received: axes={len(msg.axes)} buttons={len(msg.buttons)} '
                                   f'axes={list(msg.axes)[:8]} buttons={list(msg.buttons)[:12]}')
            if len(msg.axes) < max(self.l2_axis, self.r2_axis, self.steer_axis) + 1:
                self.get_logger().warn(
                    f'Axes size {len(msg.axes)} smaller than configured indices '
                    f'(l2={self.l2_axis}, r2={self.r2_axis}, steer={self.steer_axis}) — '
                    f'check jstest / evtest, or set l2_axis/r2_axis params. '
                    f'Fallback to buttons if enabled.'
                )
            self._first_log = False

        # Extract triggers robustly
        r2_raw = None
        l2_raw = None

        if len(msg.axes) > self.r2_axis and len(msg.axes) > self.l2_axis:
            r2_raw = float(msg.axes[self.r2_axis])
            l2_raw = float(msg.axes[self.l2_axis])
        elif self.use_buttons_fallback and len(msg.buttons) >= 8:
            # Some drivers expose L2/R2 as buttons 6/7 when axes not available
            # buttons are 0/1, convert to trigger-like 1->pressed, 0->idle then to raw -1..1
            # button pressed -> raw = -1 (pressed), idle -> raw = 1
            if len(msg.buttons) > 7:
                l2_btn = msg.buttons[6]
                r2_btn = msg.buttons[7]
                l2_raw = -1.0 if l2_btn else 1.0
                r2_raw = -1.0 if r2_btn else 1.0
                self.get_logger().debug(f'Using button fallback: L2 btn={l2_btn} R2 btn={r2_btn}')
            else:
                self.get_logger().warn('Joy axes too short and no button fallback available', throttle_duration_sec=2.0)
                return
        else:
            self.get_logger().warn(f'Joy axes size {len(msg.axes)} insufficient for triggers', throttle_duration_sec=2.0)
            return

        # Steer
        if len(msg.axes) <= self.steer_axis:
            self.get_logger().warn(f'Steer axis {self.steer_axis} out of range (size {len(msg.axes)})', throttle_duration_sec=2.0)
            return
        steer_raw = float(msg.axes[self.steer_axis])
        if self.invert_steer:
            steer_raw = -steer_raw
        # deadzone for steer
        if abs(steer_raw) < self.deadzone:
            steer_raw = 0.0

        r2_norm = self._normalize_trigger(r2_raw)
        l2_norm = self._normalize_trigger(l2_raw)

        mapped_r2 = map_value(r2_norm, 0.0, 1.0, 0, 255)
        mapped_l2 = map_value(l2_norm, 0.0, 1.0, 0, 255)
        mapped_steer = map_value(steer_raw, -1.0, 1.0, -200, 200)

        out = ImSpeed()
        # Propagate stamp for bag replay compatibility (like C++ version)
        try:
            out.header.stamp = msg.header.stamp
        except Exception:
            out.header.stamp = self.get_clock().now().to_msg()

        if mapped_l2 > 0:
            out.move = int(mapped_l2 / 10)  # 0..25
            out.which = 'L2'
        else:
            out.move = int(mapped_r2 / 10)
            out.which = 'R2'
        out.analog = int(mapped_steer)

        self.pub.publish(out)

        self._count += 1
        if self.debug_interval > 0 and self._count % self.debug_interval == 0:
            self.get_logger().info(
                f'joy->ImSpeed [{self._count}]: L2 raw={l2_raw:.2f} norm={l2_norm:.2f} | '
                f'R2 raw={r2_raw:.2f} norm={r2_norm:.2f} | steer_raw={steer_raw:.2f} -> '
                f'move={out.move} which={out.which} analog={out.analog}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PS4Reader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'PS4Reader error: {e}')
        else:
            print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
