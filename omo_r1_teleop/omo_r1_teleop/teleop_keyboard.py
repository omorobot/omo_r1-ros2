#!/usr/bin/env python3
import os
import sys
import rclpy
import select
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

MAX_LIN_VEL = 0.6   # 0.6m/s (600mm/s)
MAX_ANG_VEL = 0.5   # 0.5rad/s (28.6479degree/s)
STEP_LIN_VEL = 0.05 # 0.05m/s step (50mm/s)
STEP_ANG_VEL = 0.1  # 0.1rad/s (5.72958degree/s)

msg = '''
Control your mobile robot~~
----------------------------------------------------------------------
Moving around:
        w
    a   s   d
        x
w/x: increase/decrease linear velocity (omo_r1: ~0.6m/s)
a/d: increase/decrease angular velocity (omo_r1: ~1.0rad/s)
space, s: force stop
----------------------------------------------------------------------
CTRL+C to quit
'''

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None

    def publish(self, msg):
        self.publisher.publish(msg)

    def print(self, str_info):
        self.get_logger().info(str_info)

    def constrain(self, vel, min_vel, max_vel):
        return min(max_vel, max(min_vel, vel))

    def keyin(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def smooth_accel_decel(self, current_vel, target_vel, slop):
        if target_vel > current_vel:
            current_vel = min(target_vel, current_vel + slop)
        elif target_vel < current_vel:
            current_vel = max(target_vel, current_vel - slop)
        else:
            current_vel = target_vel
        return current_vel

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    keyin_cnt = 0
    target_lin_vel, target_ang_vel, current_lin_vel, current_ang_vel = 0.0, 0.0, 0.0, 0.0
    twist = Twist()
    try:
        print(msg)
        while True:
            key = node.keyin()
            if key == 'w' or key.lower() == 'w':
                target_lin_vel = node.constrain(target_lin_vel + STEP_LIN_VEL, -MAX_LIN_VEL, MAX_LIN_VEL)
                keyin_cnt += 1
                node.print(f'current velocity > linear {target_lin_vel:.2f},\t angular {target_ang_vel:.2f}')
            elif key == 'x' or key.lower() == 'x':
                target_lin_vel = node.constrain(target_lin_vel - STEP_LIN_VEL, -MAX_LIN_VEL, MAX_LIN_VEL)
                keyin_cnt += 1
                node.print(f'current velocity > linear {target_lin_vel:.2f},\t angular {target_ang_vel:.2f}')
            elif key == 'a' or key.lower() == 'a':
                target_ang_vel = node.constrain(target_ang_vel + STEP_ANG_VEL, -MAX_ANG_VEL, MAX_ANG_VEL)
                keyin_cnt += 1
                node.print(f'current velocity > linear {target_lin_vel:.2f},\t angular {target_ang_vel:.2f}')
            elif key == 'd' or key.lower() == 'd':
                target_ang_vel = node.constrain(target_ang_vel - STEP_ANG_VEL, -MAX_ANG_VEL, MAX_ANG_VEL)
                keyin_cnt += 1
                node.print(f'current velocity > linear {target_lin_vel:.2f},\t angular {target_ang_vel:.2f}')
            elif key == ' ' or key == 's' or key.lower() == 's':
                target_lin_vel, target_ang_vel, current_lin_vel, current_ang_vel = 0.0, 0.0, 0.0, 0.0
                keyin_cnt += 1
                node.print(f'current velocity > linear {target_lin_vel:.2f},\t angular {target_ang_vel:.2f}')
            else:
                if key == '\x03':
                    break

            if keyin_cnt == 20:
                print(msg)
                keyin_cnt = 0

            current_lin_vel = node.smooth_accel_decel(current_lin_vel, target_lin_vel, (STEP_LIN_VEL / 10.0))
            twist.linear.x, twist.linear.y, twist.linear.z = current_lin_vel, 0.0, 0.0
            current_ang_vel = node.smooth_accel_decel(current_ang_vel, target_ang_vel, (STEP_ANG_VEL / 10.0))
            twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, current_ang_vel
            node.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist.linear.x, twist.linear.y, twist.linear.z = 0.0, 0.0, 0.0
        twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, 0.0
        node.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
