#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# ==========================================
# CONFIGURATION
# ==========================================
# Robot Physical Parameters
WHEEL_SEPARATION = 0.20   # meters
WHEEL_RADIUS = 0.0325     # meters

# SPEED SCALING FACTOR (The "Software Gearbox")
# Increase this number to make the robot SLOWER
# Decrease this number to make the robot FASTER
SCALING_FACTOR = 15.0     

rospy.init_node('cmd_vel_translator')

rospy.loginfo(f"Translator started. Scaling factor: {SCALING_FACTOR}")

# Publishers for the PID Controllers
left_pub = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=1)
right_pub = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=1)

def cmd_vel_callback(msg):
    # 1. Extract Linear and Angular velocity from navigation command
    linear = msg.linear.x
    angular = msg.angular.z
    
    # 2. Calculate raw wheel velocities (rad/s) using kinematics
    # v_left = (v - w * L/2) / r
    # v_right = (v + w * L/2) / r
    raw_left = (linear - (angular * WHEEL_SEPARATION / 2.0)) / WHEEL_RADIUS
    raw_right = (linear + (angular * WHEEL_SEPARATION / 2.0)) / WHEEL_RADIUS
    
    # 3. Apply Scaling Factor to tame the speed
    scaled_left = raw_left / SCALING_FACTOR
    scaled_right = raw_right / SCALING_FACTOR
    
    # 4. Publish the scaled commands to the controllers
    left_pub.publish(Float64(scaled_left))
    right_pub.publish(Float64(scaled_right))

# Subscribe to navigation/teleop commands
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

rospy.spin()
