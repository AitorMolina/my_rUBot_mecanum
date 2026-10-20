#!/usr/bin/env python3
"""Holonomic obstacle wall follower driven directly by laser regions."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicObstacleWallFollower(Node):
    """Follow the sequence FRONT -> FRONT_RIGHT -> RIGHT -> BACK_RIGHT."""

    RIGHT = 'RIGHT'
    FRONT = 'FRONT'
    FRONT_RIGHT = 'FRONT_RIGHT'
    BACK_RIGHT = 'BACK_RIGHT'

    # Fixed controller constants: they are deliberately not ROS parameters.
    FRONT_TRIGGER_FACTOR = 1.5
    DISTANCE_GAIN = 0.8
    DIAGONAL_FACTOR = 0.5
    REGION_CONFIRMATIONS = 3
    ROBUST_RAYS = 3

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        # Minimum set of parameters needed to tune the real robot.
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.25).value
        )
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.30).value
        )
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 0.60).value
        )
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )

        self.state = self.RIGHT
        self.short_obstacle = True
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None
        self.last_status = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            'Holonomic obstacle wall follower ready: '
            f'follow_distance={self.follow_distance:.2f} m, '
            f'max_front_length={self.max_front_length:.2f} m'
        )

    def publish_command(self):
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.odom_pose = (position.x, position.y, yaw)

    @classmethod
    def region_measurements(cls, scan):
        """Return a robust distance for each region.

        Angles are normalised so scans expressed as either [-pi, pi] or
        [0, 2*pi] work in the same way. The mean of the nearest three valid
        rays prevents one noisy hardware return from controlling the state.
        """
        regions = {
            cls.FRONT: [],
            cls.FRONT_RIGHT: [],
            cls.RIGHT: [],
            cls.BACK_RIGHT: [],
        }

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            raw_angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
            angle_deg = math.degrees(angle)

            if -20.0 <= angle_deg <= 20.0:
                regions[cls.FRONT].append(distance)
            elif -70.0 <= angle_deg < -20.0:
                regions[cls.FRONT_RIGHT].append(distance)
            elif -110.0 <= angle_deg < -70.0:
                regions[cls.RIGHT].append(distance)
            elif -160.0 <= angle_deg < -110.0:
                regions[cls.BACK_RIGHT].append(distance)

        measurements = {}
        for name, values in regions.items():
            nearest = sorted(values)[:cls.ROBUST_RAYS]
            measurements[name] = (
                sum(nearest) / len(nearest) if nearest else math.inf
            )
        return measurements

    def closest_region(self, measurements):
        visible = {
            name: distance
            for name, distance in measurements.items()
            if distance < self.detection_limit
        }
        return min(visible, key=visible.get) if visible else None

    def confirmed_region(self, measurements):
        """Debounce region changes over several consecutive laser scans."""
        candidate = self.closest_region(measurements)
        if candidate == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = candidate
            self.region_candidate_count = 1

        if self.region_candidate_count >= self.REGION_CONFIRMATIONS:
            self.stable_region = candidate
        return self.stable_region

    def lateral_progress(self):
        """Distance travelled along the initial robot lateral axis."""
        if self.odom_pose is None or self.front_start_pose is None:
            return None

        start_x, start_y, start_yaw = self.front_start_pose
        current_x, current_y, _ = self.odom_pose
        dx = current_x - start_x
        dy = current_y - start_y
        return abs(
            -math.sin(start_yaw) * dx
            + math.cos(start_yaw) * dy
        )

    def change_state(self, new_state, reason):
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(f'STATE {new_state}: {reason}')

    def follow_right(self, measurements, closest, cmd):
        front_distance = measurements[self.FRONT]
        right_distance = measurements[self.RIGHT]
        front_trigger = self.FRONT_TRIGGER_FACTOR * self.follow_distance

        if closest == self.FRONT and front_distance < front_trigger:
            if self.odom_pose is None:
                self.report('RIGHT: frontal obstacle; waiting for /odom')
                return

            self.front_start_pose = self.odom_pose
            self.short_obstacle = True
            self.change_state(self.FRONT, 'frontal obstacle detected')
            cmd.linear.y = self.lateral_speed
            return

        if closest == self.BACK_RIGHT:
            self.change_state(
                self.BACK_RIGHT,
                'minimum moved from RIGHT to BACK_RIGHT',
            )
            self.follow_back_right(measurements, closest, cmd)
            return

        if right_distance < self.detection_limit:
            distance_error = right_distance - self.follow_distance
            cmd.linear.x = self.forward_speed
            cmd.linear.y = clamp(
                -self.DISTANCE_GAIN * distance_error,
                -self.lateral_speed,
                self.lateral_speed,
            )
            self.report(f'RIGHT: distance={right_distance:.2f} m')
        else:
            cmd.linear.x = 0.5 * self.forward_speed
            cmd.linear.y = -self.lateral_speed
            self.report('RIGHT: searching for wall')

    def follow_front(self, measurements, closest, cmd):
        progress = self.lateral_progress()
        if progress is None:
            self.report('FRONT: odometry unavailable; stopped')
            return

        if progress >= self.max_front_length:
            self.short_obstacle = False

        if closest == self.FRONT_RIGHT:
            obstacle_type = (
                'short obstacle' if self.short_obstacle else 'long wall'
            )
            self.change_state(
                self.FRONT_RIGHT,
                f'{obstacle_type}; progress={progress:.2f} m',
            )
            self.follow_front_right(measurements, closest, cmd)
            return

        if not self.short_obstacle:
            if closest == self.RIGHT:
                self.change_state(self.RIGHT, 'long front wall reached RIGHT')
                self.follow_right(measurements, closest, cmd)
            else:
                cmd.angular.z = self.turn_speed
                self.report(
                    f'FRONT: long obstacle, turning; progress={progress:.2f} m'
                )
            return

        # Graceful handling if a low-rate scan skips FRONT_RIGHT.
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'short obstacle reached RIGHT')
            self.follow_right(measurements, closest, cmd)
            return

        cmd.linear.y = self.lateral_speed
        self.report(
            f'FRONT: lateral progress={progress:.2f} m, closest={closest}'
        )

    def follow_front_right(self, measurements, closest, cmd):
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'obstacle moved to RIGHT')
            self.follow_right(measurements, closest, cmd)
            return

        if not self.short_obstacle:
            cmd.angular.z = self.turn_speed
            self.report(
                f'FRONT_RIGHT: long wall, turning; closest={closest}'
            )
            return

        if closest == self.BACK_RIGHT:
            self.change_state(
                self.BACK_RIGHT,
                'obstacle moved directly to BACK_RIGHT',
            )
            self.follow_back_right(measurements, closest, cmd)
            return

        cmd.linear.x = self.forward_speed
        cmd.linear.y = self.DIAGONAL_FACTOR * self.lateral_speed
        self.report(f'FRONT_RIGHT: diagonal +vx/+vy, closest={closest}')

    def follow_back_right(self, measurements, closest, cmd):
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'right wall recovered')
            self.follow_right(measurements, closest, cmd)
            return

        cmd.linear.x = self.forward_speed
        cmd.linear.y = -self.DIAGONAL_FACTOR * self.lateral_speed
        self.report(f'BACK_RIGHT: diagonal +vx/-vy, closest={closest}')

    def scan_callback(self, scan):
        measurements = self.region_measurements(scan)
        closest = self.confirmed_region(measurements)
        cmd = Twist()

        if self.state == self.RIGHT:
            self.follow_right(measurements, closest, cmd)
        elif self.state == self.FRONT:
            self.follow_front(measurements, closest, cmd)
        elif self.state == self.FRONT_RIGHT:
            self.follow_front_right(measurements, closest, cmd)
        else:
            self.follow_back_right(measurements, closest, cmd)

        self.command = cmd

    def report(self, status):
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop(self):
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = HolonomicObstacleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
