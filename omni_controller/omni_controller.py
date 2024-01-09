#!/usr/bin/env python3

import rclpy
import threading
import serial
import time
import math
import tf_transformations

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from tf2_ros import TransformBroadcaster
from omni_controller.robot import Robot, compute_motor_velocities

SPEED_TO_ANGLE_RATIO = 3228

X_SPEED_FACTOR_ODOM = 0.00035
Y_SPEED_FACTOR_ODOM = 0.00103
Z_SPEED_FACTOR_ODOM = 0.00035


class DriveController(Node):

    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info('Starting Drive Controller Node')

        self.saved_time_rx_in_s = time.time()

        self.last_wheel_front_left_speed    = 0
        self.last_wheel_front_right_speed   = 0
        self.last_wheel_rear_left_speed     = 0
        self.last_wheel_rear_right_speed    = 0

        self.serial_port_1 = serial.Serial()
        self.serial_port_1.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0'
        self.serial_port_1.baudrate = 57600
        self.serial_port_1.timeout = 1000
        self.serial_port_1.open()

        self.serial_port_2 = serial.Serial()
        self.serial_port_2.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0'
        self.serial_port_2.baudrate = 57600
        self.serial_port_2.timeout = 1000
        self.serial_port_2.open()

        wheel_radius = 0.05
        wheel_base = 0.42
        track_width = 0.48
        max_v = 0.8
        max_w = 1.0

        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.apply_velocity, qos_profile=qos_profile_system_default)
        self.joints_publisher = self.create_publisher(JointState, '/joint_states', qos_profile=qos_profile_system_default)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', qos_profile=qos_profile_services_default)
        self.odom_broadcaster = TransformBroadcaster(self, qos_profile_services_default)

        if self.declare_parameter('port_control_wheel_front', rclpy.Parameter.Type.STRING):
            self.serial_port_1.port = self.get_parameter('port_control_wheel_front').value
        if self.declare_parameter('port_control_wheel_back', rclpy.Parameter.Type.STRING):
            self.serial_port_2.port = self.get_parameter('port_control_wheel_back').value
        if self.declare_parameter('baudrate_port_wheel_front', rclpy.Parameter.Type.INTEGER):
            self.serial_port_1.baudrate = self.get_parameter('baudrate_port_wheel_front').value
        if self.declare_parameter('baudrate_port_wheel_back', rclpy.Parameter.Type.INTEGER):
            self.serial_port_2.baudrate = self.get_parameter('baudrate_port_wheel_back').value
        if self.declare_parameter('timeout', rclpy.Parameter.Type.INTEGER):
            self.serial_port_1.timeout = self.get_parameter('timeout').value
            self.serial_port_2.timeout = self.get_parameter('timeout').value
        if self.declare_parameter('wheel_radius', rclpy.Parameter.Type.DOUBLE):
            wheel_radius = self.get_parameter('wheel_radius').value
        if self.declare_parameter('wheel_base', rclpy.Parameter.Type.DOUBLE):
            wheel_base = self.get_parameter('wheel_base').value
        if self.declare_parameter('track_width', rclpy.Parameter.Type.DOUBLE):
            wheel_base = self.get_parameter('track_width').value

        self.get_logger().info(f"port wheel front: {self.serial_port_1.port}")
        self.get_logger().info(f"port wheel back: {self.serial_port_2.port}")
        self.get_logger().info(f"baudrate wheel front: {self.serial_port_1.baudrate}")
        self.get_logger().info(f"baudrate wheel back: {self.serial_port_2.baudrate}")
        self.get_logger().info(f"timeout: {self.serial_port_1.timeout}")
        self.get_logger().info(f"wheel radius: {wheel_radius}")
        self.get_logger().info(f"wheel base: {wheel_base}")
        self.get_logger().info(f"track width: {track_width}")

        self.robot = Robot(wheel_base, track_width, wheel_radius, max_v, max_w)

        self.wheel_front_left_rotation  = 0.0
        self.wheel_front_right_rotation = 0.0
        self.wheel_rear_left_rotation   = 0.0
        self.wheel_rear_right_rotation  = 0.0

        self.linear_x_position  = 0.0
        self.linear_y_position  = 0.0
        self.angular_z_position = 0.0

        self.read_thread = threading.Thread(target=self.read_thread_function)
        self.read_thread.start()

    def apply_velocity(self, msg):
        input = [msg.linear.x, msg.linear.y, msg.angular.z]
        result = compute_motor_velocities(input, self.robot)
        command_string_f = 'o {:.0f} {:.0f}\r'.format(result[0], result[1])
        command_string_b = 'o {:.0f} {:.0f}\r'.format(result[2], result[3])
        command_bytes_f = bytes(command_string_f, encoding='ascii')
        command_bytes_b = bytes(command_string_b, encoding='ascii')
        self.serial_port_1.write(command_bytes_f)
        self.serial_port_2.write(command_bytes_b)
        self.get_logger().info('Sending : ' + str(command_bytes_f) + ' ' + str(command_bytes_b))

    def get_rotation_in_rad(self, wheel_rotation):
        return wheel_rotation % (2 * math.pi) - math.pi

    def publish_wheels_state(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        self.wheel_front_left_rotation  += wheel_front_left_speed  / SPEED_TO_ANGLE_RATIO
        self.wheel_front_right_rotation += wheel_front_right_speed / SPEED_TO_ANGLE_RATIO
        self.wheel_rear_left_rotation   += wheel_rear_left_speed   / SPEED_TO_ANGLE_RATIO
        self.wheel_rear_right_rotation  += wheel_rear_right_speed  / SPEED_TO_ANGLE_RATIO

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_back_left_joint', 'wheel_back_right_joint']
        joint_states.position = [
            self.get_rotation_in_rad(self.wheel_front_left_rotation), 
            self.get_rotation_in_rad(self.wheel_front_right_rotation), 
            self.get_rotation_in_rad(self.wheel_rear_left_rotation), 
            self.get_rotation_in_rad(self.wheel_rear_right_rotation)]
        self.joints_publisher.publish(joint_states)
        return

    def publish_odom(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        linear_x_velocity  =  wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed + wheel_rear_right_speed
        linear_y_velocity  = -wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed - wheel_rear_right_speed
        angular_z_velocity = -wheel_front_left_speed + wheel_front_right_speed - wheel_rear_left_speed + wheel_rear_right_speed

        linear_x_velocity  *= X_SPEED_FACTOR_ODOM
        linear_y_velocity  *= Y_SPEED_FACTOR_ODOM
        angular_z_velocity *= Z_SPEED_FACTOR_ODOM

        if (wheel_front_left_speed != self.last_wheel_front_left_speed or wheel_front_right_speed != self.last_wheel_front_right_speed or wheel_rear_left_speed != self.last_wheel_rear_left_speed or wheel_rear_right_speed != self.last_wheel_rear_right_speed):

            self.get_logger().info('Received: {} {} {} {}'.format(int(wheel_front_left_speed), int(wheel_front_right_speed), int(wheel_rear_left_speed), int(wheel_rear_right_speed)))
            self.get_logger().info('X: :{:.2f} / Y: {:.2f} / Z: {:.2f}'.format(linear_x_velocity, linear_y_velocity, angular_z_velocity))

            self.last_wheel_front_left_speed  = wheel_front_left_speed
            self.last_wheel_front_right_speed = wheel_front_right_speed
            self.last_wheel_rear_left_speed   = wheel_rear_left_speed
            self.last_wheel_rear_right_speed  = wheel_rear_right_speed

        current_time_rx_in_s = time.time()
        delta_time_in_s = current_time_rx_in_s - self.saved_time_rx_in_s
        self.saved_time_rx_in_s = current_time_rx_in_s

        delta_x = delta_time_in_s * (linear_x_velocity * math.cos(self.angular_z_position) - linear_y_velocity * math.sin(self.angular_z_position))
        delta_y = delta_time_in_s * (linear_x_velocity * math.sin(self.angular_z_position) + linear_y_velocity * math.cos(self.angular_z_position))
        delta_z = delta_time_in_s * angular_z_velocity

        self.linear_x_position  += delta_x
        self.linear_y_position  += delta_y
        self.angular_z_position += delta_z

        tf_quat = tf_transformations.quaternion_from_euler(0, 0, self.angular_z_position)
        msg_quat = Quaternion(x=tf_quat[0], y=tf_quat[1], z=tf_quat[2], w=tf_quat[3])

        odom_transform = TransformStamped()
        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'
        odom_transform.transform.translation.x = self.linear_x_position
        odom_transform.transform.translation.y = self.linear_y_position
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = msg_quat
        self.odom_broadcaster.sendTransform(odom_transform)

        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = self.linear_x_position
        odometry.pose.pose.position.y = self.linear_y_position
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = msg_quat
        odometry.twist.twist.linear.x = linear_x_velocity
        odometry.twist.twist.linear.y = linear_y_velocity
        odometry.twist.twist.angular.z = angular_z_velocity
        self.odom_publisher.publish(odometry)

    def read_thread_function(self):
        self.get_logger().info('Start reading encoder')
        while True:
            self.serial_port_2.write(bytes('a\r', encoding='ascii'))
            self.serial_port_1.write(bytes('a\r', encoding='ascii'))
            msg_2 = self.serial_port_2.readline().decode('utf-8', 'ignore')
            msg_1 = self.serial_port_1.readline().decode('utf-8', 'ignore')
            msg_2_split = msg_2.split()
            msg_1_split = msg_1.split()
            if len(msg_1_split) == 2 and len(msg_2_split) == 2:
                self.publish_wheels_state(int(msg_1_split[0]), int(msg_1_split[1]), int(msg_2_split[0]), int(msg_2_split[1]))
                self.publish_odom(int(msg_1_split[0]), int(msg_1_split[1]), int(msg_2_split[0]), int(msg_1_split[1]))

    


def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveController()
    try:
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
    except BaseException:
        print('Stopped by exception')
        raise
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
