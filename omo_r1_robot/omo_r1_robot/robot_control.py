#!/usr/bin/env python3
import math
import rclpy
import transforms3d
from .packet_handler import *
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Pose

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.declare_parameters(
            namespace='',
            parameters=[('port.name', Parameter.Type.STRING),
                        ('port.baudrate', Parameter.Type.INTEGER),
                        ('wheel.separation', Parameter.Type.DOUBLE),
                        ('wheel.radius', Parameter.Type.DOUBLE),
                        ('motor.gear_ratio', Parameter.Type.DOUBLE),
                        ('motor.max_lin_vel', Parameter.Type.DOUBLE),
                        ('motor.max_ang_vel', Parameter.Type.DOUBLE),
                        ('sensor.enc_pulse', Parameter.Type.DOUBLE),
                        ('sensor.use_imu', Parameter.Type.BOOL)
            ]
        )
        
        port = self.get_parameter_or('port.name', Parameter('port.name', Parameter.Type.STRING, '/dev/ttyMCU')).get_parameter_value().string_value
        baudrate = self.get_parameter_or('port.baudrate', Parameter('port.baudrate', Parameter.Type.INTEGER, 115200)).get_parameter_value().integer_value
        self.wheel_separation = self.get_parameter_or('wheel.separation', Parameter('wheel.separation', Parameter.Type.DOUBLE, 0.57)).get_parameter_value().double_value
        self.wheel_radius = self.get_parameter_or('wheel.radius', Parameter('wheel.radius', Parameter.Type.DOUBLE, 0.1)).get_parameter_value().double_value
        self.motor_gear_ratio = self.get_parameter_or('motor.gear_ratio', Parameter('motor.gear_ratio', Parameter.Type.DOUBLE, 15.0)).get_parameter_value().double_value
        self.motor_max_lin_vel = self.get_parameter_or('motor.max_lin_vel', Parameter('motor.max_lin_vel', Parameter.Type.DOUBLE, 0.6)).get_parameter_value().double_value
        self.motor_max_ang_vel = self.get_parameter_or('motor.max_ang_vel', Parameter('motor.max_ang_vel', Parameter.Type.DOUBLE, 1.0)).get_parameter_value().double_value
        self.enc_pulse = self.get_parameter_or('sensor.enc_pulse', Parameter('sensor.enc_pulse', Parameter.Type.DOUBLE, 60000.0)).get_parameter_value().double_value
        self.use_imu = self.get_parameter_or('sensor.use_imu', Parameter('sensor.use_imu', Parameter.Type.BOOL, False)).get_parameter_value().bool_value
        # self.distance_per_pulse = 2 * math.pi * self.wheel_radius / self.enc_pulse / self.motor_gear_ratio
        self.print(f'port.name:\t\t{port}')
        self.print(f'port.baudrate:\t\t{baudrate}')
        self.print(f'wheel.separation:\t{self.wheel_separation}')
        self.print(f'wheel.radius:\t\t{self.wheel_radius}')
        self.print(f'motor.gear_ratio:\t{self.motor_gear_ratio}')
        self.print(f'motor.max_lin_vel:\t{self.motor_max_lin_vel}')
        self.print(f'motor.max_ang_vel:\t{self.motor_max_ang_vel}')
        self.print(f'sensor.enc_pulse:\t{self.enc_pulse}')
        self.print(f'sensor.use_imu:\t{self.use_imu}')
        # self.print(f'distance per pulse:\t{self.distance_per_pulse}')

        self.odom_pose = Odometry()
        self.odom_pose_pre_stamp = self.get_clock().now()                                                   # self.odom_pose.header.stamp = self.get_clock().now()

        qos_profile = QoSProfile(depth=10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)   # linear/angular velocity
        self.pub_joint_state = self.create_publisher(JointState, 'joint_states', qos_profile)               # axis position/velocity
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_profile)                                # linear/angular velocity, position/orientation
        # self.pub_pose = self.create_publisher(Pose, 'pose', qos_profile)
        self.tf_bc = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.update_robot)                                             # 10ms timer

        self.ph = PacketHandler(port, baudrate)
        self.ph.set_periodic_info(50)

    def print(self, str_info):
        self.get_logger().info(str_info)

    def cmd_vel_callback(self, msg):
        v = max(-self.motor_max_lin_vel, min(self.motor_max_lin_vel, msg.linear.x))
        w = max(-self.motor_max_ang_vel, min(self.motor_max_ang_vel, msg.angular.z))
        self.ph.vw_command(v * 1000, w * 1000)                                                              # form m/s & rad/s to mm/s & mrad/s

    def update_robot(self):
        self.ph.read_packet()
        odo_lh = self.ph._odo[0]
        odo_rh = self.ph._odo[1]
        lin_vel = self.ph._vw[0]
        ang_vel = self.ph._vw[1]
        # vel_z = self.ph._gyro[2]
        # pose_roll = self.ph._pose[0]
        # pose_pitch = self.ph._pose[1]
        # pose_yaw = self.ph._pose[2]
        self.update_odometry(lin_vel, ang_vel)
        self.update_jointstate(odo_lh, odo_rh, lin_vel, ang_vel)
        # self.update_pose(pose_roll, pose_pitch, pose_yaw)

    def update_odometry(self, lin_vel, ang_vel):
        lin_vel /= 1000.                                                                                    # form mm/s & mrad/s to m/s & rad/s
        ang_vel /= 1000.
        self.odom_pose_stamp = self.get_clock().now()
        dt = (self.odom_pose_stamp - self.odom_pose_pre_stamp).nanoseconds * 1e-9
        self.odom_pose_pre_stamp = self.odom_pose_stamp
        self.odom_pose.twist.twist.angular.z += ang_vel * dt
        self.odom_pose.pose.pose.position.x += lin_vel * math.cos(self.odom_pose.twist.twist.angular.z) * dt
        self.odom_pose.pose.pose.position.y += lin_vel * math.sin(self.odom_pose.twist.twist.angular.z) * dt
        q = transforms3d.euler.euler2quat(0, 0, self.odom_pose.twist.twist.angular.z)
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.pose.pose.position.x = self.odom_pose.pose.pose.position.x
        odometry.pose.pose.position.y = self.odom_pose.pose.pose.position.y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = q[1]
        odometry.pose.pose.orientation.y = q[2]
        odometry.pose.pose.orientation.z = q[3]
        odometry.pose.pose.orientation.w = q[0]
        odometry.child_frame_id = "base_footprint"
        odometry.twist.twist.linear.x = lin_vel
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = ang_vel
        self.pub_odom.publish(odometry)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = odometry.header.stamp
        odom_tf.header.frame_id = odometry.header.frame_id
        odom_tf.child_frame_id = odometry.child_frame_id
        odom_tf.transform.translation.x = odometry.pose.pose.position.x
        odom_tf.transform.translation.y = odometry.pose.pose.position.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = odometry.pose.pose.orientation
        self.tf_bc.sendTransform(odom_tf)

    def update_jointstate(self, odo_lh, odo_rh, lin_vel, ang_vel):
        odo_lh /= 1000.                                                                                     # form mm to m
        odo_rh /= 1000.
        lin_vel /= 1000.                                                                                    # form mm/s & mrad/s to m/s & rad/s
        ang_vel /= 1000.
        wheel_lh_pos = odo_lh / self.wheel_radius                                                           # wheel angle (rad)
        wheel_rh_pos = odo_rh / self.wheel_radius
        wheel_lh_vel = (lin_vel - (self.wheel_separation / 2.0) * ang_vel) / self.wheel_radius              # wheel angular velocity (rad/s)
        wheel_rh_vel = (lin_vel + (self.wheel_separation / 2.0) * ang_vel) / self.wheel_radius
        jointstate = JointState()
        jointstate.header.frame_id = "base_link"
        jointstate.header.stamp = self.get_clock().now().to_msg()
        jointstate.name = ['wheel_left_joint', 'wheel_right_joint']
        jointstate.position = [wheel_lh_pos, wheel_rh_pos]
        jointstate.velocity = [wheel_lh_vel, wheel_rh_vel]
        jointstate.effort = []
        self.pub_joint_state.publish(jointstate)

    # def update_pose(self, pose_roll, pose_pitch, pose_yaw):
    #     pose = Pose()
    #     pose.orientation.x = pose_roll
    #     pose.orientation.y = pose_pitch
    #     pose.orientation.z = pose_yaw
    #     self.pub_pose.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.ph.close_port()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
