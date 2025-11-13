#!/usr/bin/env python3

import rospy
import pigpio
import math
import time

# ROS Messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# For I2C communication with IMU
import board
import adafruit_mpu6050
import py_qmc5883l

# =================================================================
# PIN DEFINITIONS & CONFIGURATION
# =================================================================
# Motor Pins (using GPIO numbers)
MOTOR_LEFT_PWM = 12
MOTOR_LEFT_DIR1 = 20
MOTOR_LEFT_DIR2 = 21

MOTOR_RIGHT_PWM = 13
MOTOR_RIGHT_DIR1 = 23
MOTOR_RIGHT_DIR2 = 24

# Encoder Pins (using GPIO numbers)
ENCODER_LEFT_A = 5
ENCODER_LEFT_B = 6
ENCODER_RIGHT_A = 26
ENCODER_RIGHT_B = 16

# Robot Physical Parameters
WHEEL_DIAMETER = 0.070
WHEEL_BASE = 0.180
ENCODER_TICKS_PER_REV = 120240
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV

# PWM Configuration
PWM_FREQ = 5000
PWM_RANGE = 255

# =================================================================
# GLOBAL VARIABLES
# =================================================================
pi = pigpio.pi()
encoder_left_count = 0
encoder_right_count = 0
last_odom_time = None
odom_x = 0.0
odom_y = 0.0
odom_theta = 0.0

# =================================================================
# INTERRUPT SERVICE ROUTINES (ENCODERS)
# =================================================================
def encoder_left_isr(gpio, level, tick):
    global encoder_left_count
    if pi.read(ENCODER_LEFT_B):
        encoder_left_count += 1
    else:
        encoder_left_count -= 1

def encoder_right_isr(gpio, level, tick):
    global encoder_right_count
    if pi.read(ENCODER_RIGHT_B):
        encoder_right_count -= 1
    else:
        encoder_right_count += 1

# =================================================================
# MOTOR CONTROL FUNCTION
# =================================================================
def set_motor_speed(motor_pwm_pin, motor_dir1_pin, motor_dir2_pin, speed):
    direction = speed >= 0
    pwm_value = int(abs(speed))
    if pwm_value > PWM_RANGE:
        pwm_value = PWM_RANGE
    if direction:
        pi.write(motor_dir1_pin, 1)
        pi.write(motor_dir2_pin, 0)
    else:
        pi.write(motor_dir1_pin, 0)
        pi.write(motor_dir2_pin, 1)
    pi.set_PWM_dutycycle(motor_pwm_pin, pwm_value)

# =================================================================
# ROS CALLBACK FOR /cmd_vel
# =================================================================
def cmd_vel_callback(twist_msg):
    linear_vel = twist_msg.linear.x
    angular_vel = twist_msg.angular.z
    wheel_radius = WHEEL_DIAMETER / 2.0
    vel_left_rad_s = (linear_vel - (angular_vel * WHEEL_BASE / 2.0)) / wheel_radius
    vel_right_rad_s = (linear_vel + (angular_vel * WHEEL_BASE / 2.0)) / wheel_radius
    pwm_left = vel_left_rad_s * 50
    pwm_right = vel_right_rad_s * 50
    set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, pwm_left)
    set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, pwm_right)

# =================================================================
# MAIN FUNCTION
# =================================================================
def main():
    global last_odom_time, encoder_left_count, encoder_right_count, odom_x, odom_y, odom_theta

    rospy.init_node('pi_controller_node')

    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    odom_pub = rospy.Publisher('wheel_encoder', Odometry, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    #qmc = py_qmc5883l.QMC5883L()

    for pin in [MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2]:
        pi.set_mode(pin, pigpio.OUTPUT)
    for pin in [MOTOR_LEFT_PWM, MOTOR_RIGHT_PWM]:
        pi.set_PWM_frequency(pin, PWM_FREQ)
        pi.set_PWM_range(pin, PWM_RANGE)
    for pin in [ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B]:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)

    pi.callback(ENCODER_LEFT_A, pigpio.RISING_EDGE, encoder_left_isr)
    pi.callback(ENCODER_RIGHT_A, pigpio.RISING_EDGE, encoder_right_isr)

    rospy.loginfo("Pi Controller Node Started.")

    rate = rospy.Rate(20)
    last_odom_time = rospy.Time.now()
    last_left_count = 0
    last_right_count = 0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        accel = mpu.acceleration
        gyro = mpu.gyro
        mag = [0,0,0]
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = accel
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = mag[0], mag[1], mag[2], 0
        imu_pub.publish(imu_msg)

        dt = (current_time - last_odom_time).to_sec()
        current_left_count = encoder_left_count
        current_right_count = encoder_right_count
        delta_left = current_left_count - last_left_count
        delta_right = current_right_count - last_right_count
        last_left_count, last_right_count = current_left_count, current_right_count

        dist_left = delta_left * METERS_PER_TICK
        dist_right = delta_right * METERS_PER_TICK
        delta_dist = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / WHEEL_BASE

        odom_x += delta_dist * math.cos(odom_theta)
        odom_y += delta_dist * math.sin(odom_theta)
        odom_theta += delta_theta

        linear_vel_x = delta_dist / dt if dt > 0 else 0
        angular_vel_z = delta_theta / dt if dt > 0 else 0

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = odom_x
        odom_msg.pose.pose.position.y = odom_y

        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, odom_theta)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q

        odom_msg.twist.twist.linear.x = linear_vel_x
        odom_msg.twist.twist.angular.z = angular_vel_z
        odom_pub.publish(odom_msg)

        last_odom_time = current_time
        rate.sleep()

def cleanup():
    rospy.loginfo("Stopping motors and cleaning up GPIO.")
    set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, 0)
    set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, 0)
    pi.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cleanup()
