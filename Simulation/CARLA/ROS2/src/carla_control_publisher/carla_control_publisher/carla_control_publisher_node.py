import rclpy
from rclpy.node import Node

import numpy as np
import math
import carla
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float64

MAX_ACCELERATION = 1.5  # m/s
MAX_DECELERATION = 3.0  # m/s


class CarlaControlPublisher(Node):
    def __init__(self):
        super().__init__('carla_control_publisher')
        self.steering_sub_ = self.create_subscription(Float64, '/vehicle/steering_cmd', self.steering_callback, 1)
        self.throttle_sub_ = self.create_subscription(Float64, '/vehicle/throttle_cmd', self.throttle_callback, 1)
        self.control_pub_ = self.create_publisher(CarlaEgoVehicleControl, "/carla/hero/vehicle_control_cmd", 1)

        self.steering_angle_cmd = 0.0
        self.throttle_cmd = 0.0

        # Publish only once BOTH a fresh steering and a fresh throttle have
        # arrived. Each planning cycle in VisionPilot emits one steering + one
        # throttle together, so this fires once per cycle, at the planner's own
        # rate, with no zero-order-hold delay.
        self.have_steering = False
        self.have_throttle = False

        # Failsafe watchdog: if upstream stalls, don't keep applying the last
        # (possibly full-lock) command forever. See note below.
        self.last_cmd_time = self.get_clock().now()
        # self.watchdog = self.create_timer(0.2, self.watchdog_callback)

    def publish_control(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = self.throttle_cmd / MAX_ACCELERATION if self.throttle_cmd > 0 else 0.0
        msg.steer = np.clip((180.0 / np.pi) * (self.steering_angle_cmd / 70.0), -1.0, 1.0)  # vehicle specific
        msg.brake = -self.throttle_cmd / MAX_DECELERATION if self.throttle_cmd < 0 else 0.0
        msg.hand_brake = False
        msg.reverse = False
        msg.manual_gear_shift = False
        msg.gear = 1
        self.control_pub_.publish(msg)
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info(f'Steering published: {msg.steer}')
        self.get_logger().info(f'Throttle published: {msg.throttle}')

    def try_publish(self):
        if self.have_steering and self.have_throttle:
            self.publish_control()
            self.have_steering = False
            self.have_throttle = False

    def steering_callback(self, msg):
        self.get_logger().info(f'Steering command received: {msg.data}')
        self.steering_angle_cmd = msg.data
        self.have_steering = True
        self.try_publish()

    def throttle_callback(self, msg):
        self.get_logger().info(f'Throttle command received: {msg.data}')
        self.throttle_cmd = msg.data
        self.have_throttle = True
        self.try_publish()

    def watchdog_callback(self):
        # If no fresh pair has been published recently, command a safe stop
        # (zero throttle, no brake) rather than latching the last steering.
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > 0.2:
            msg = CarlaEgoVehicleControl()
            msg.throttle = 0.0
            msg.steer = 0.0
            msg.brake = 0.0
            msg.gear = 1
            self.control_pub_.publish(msg)
            self.get_logger().warn(f'Watchdog: no fresh command for {dt:.2f}s — safing')


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
