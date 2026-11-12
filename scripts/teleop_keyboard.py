#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

# Speed limits, tune here
speed_x = 0.3  # forward/back, m/s
speed_y = 0.3  # lateral, m/s
speed_w = 0.6  # yaw, rad/s


# Read one key, '' if none
def getKey():
    # Wait at most 0.1 s
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        return sys.stdin.read(1)
    return ''


if __name__ == '__main__':
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # cbreak: no Enter needed
    tty.setcbreak(sys.stdin.fileno())

    rospy.init_node('teleop_keyboard')

    # /cmd_vel publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Command, one axis per key
    msg = Twist()

    # Key map
    print('键盘控制底盘：')
    print('  w / s : 前进 / 后退')
    print('  a / d : 左移 / 右移')
    print('  q / e : 左转 / 右转')
    print('  空格  : 停车')
    print('注意：按一下就按那个速度一直走，停车按空格，退出按 Ctrl-C')

    try:
        while not rospy.is_shutdown():
            key = getKey()

            # Update the matching axis
            if key == 'w':
                msg.linear.x = speed_x
            elif key == 's':
                msg.linear.x = -speed_x
            elif key == 'a':
                msg.linear.y = speed_y
            elif key == 'd':
                msg.linear.y = -speed_y
            elif key == 'q':
                msg.angular.z = speed_w
            elif key == 'e':
                msg.angular.z = -speed_w
            elif key == ' ':
                msg = Twist()  # reset all axes

            # 20 Hz so stop takes effect
            pub.publish(msg)
            rospy.sleep(0.05)
    finally:
        # Stop, restore terminal
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
