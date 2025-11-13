#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Movement instructions
msg = """
---------------------------
ROBOT KEYBOARD CONTROL
---------------------------
Moving around:
   w
 a s d

w : forward
s : backward
a : rotate left
d : rotate right
space : stop

q/e : increase/decrease linear speed by 10%
z/c : increase/decrease angular speed by 10%

CTRL-C to quit
---------------------------
"""

# Key bindings
moveBindings = {
    'w': (1, 0),   # forward
    's': (-1, 0),  # backward
    'a': (0, 1),   # rotate left
    'd': (0, -1),  # rotate right
}

speedBindings = {
    'q': (1.1, 1.0),   # increase linear speed
    'e': (0.9, 1.0),   # decrease linear speed
    'z': (1.0, 1.1),   # increase angular speed
    'c': (1.0, 0.9),   # decrease angular speed
}

def getKey():
    """Get keyboard input without requiring Enter key."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """Display current speed settings."""
    return f"Current: speed {speed:.2f}\tturn {turn:.2f}"

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Initial speed settings
    linear_speed = 0.2   # m/s
    angular_speed = 0.5  # rad/s
    
    x = 0
    th = 0
    status = 0
    
    try:
        print(msg)
        print(vels(linear_speed, angular_speed))
        
        while True:
            key = getKey()
            
            # Movement keys
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            
            # Speed adjustment keys
            elif key in speedBindings.keys():
                linear_speed = linear_speed * speedBindings[key][0]
                angular_speed = angular_speed * speedBindings[key][1]
                print(vels(linear_speed, angular_speed))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            
            # Stop key (space)
            elif key == ' ':
                x = 0
                th = 0
            
            # Quit
            elif key == '\x03':  # Ctrl+C
                break
            
            else:
                x = 0
                th = 0
            
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = x * linear_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * angular_speed
            pub.publish(twist)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
