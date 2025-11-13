#!/usr/bin/env python3
import rospy
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher')

        # Wheel parameters (adjust if different)
        self.wheel_sep = 0.20     # distance between wheels (meters)
        self.wheel_rad = 0.0325   # wheel radius (meters)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.initialized = False

        # Low-pass filter memory
        self.prev_d_left = 0.0
        self.prev_d_right = 0.0

        # ROS Publishers/Subscribers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)

        rospy.loginfo("✅ odom_publisher node started and waiting for joint_states...")

    def joint_states_callback(self, msg):
        try:
            left_idx = msg.name.index("left_wheel_joint")
            right_idx = msg.name.index("right_wheel_joint")
            curr_left_pos = msg.position[left_idx]
            curr_right_pos = msg.position[right_idx]
        except (ValueError, IndexError):
            rospy.logwarn_throttle(5, "⚠️ Waiting for correct joint names in joint_states...")
            return

        # Initialize once
        if not self.initialized:
            self.prev_left_pos = curr_left_pos
            self.prev_right_pos = curr_right_pos
            self.initialized = True
            rospy.loginfo("✅ Odometry initialized with first wheel readings.")
            return

        # Δwheel rotation (radians)
        d_left_rad = curr_left_pos - self.prev_left_pos
        d_right_rad = curr_right_pos - self.prev_right_pos

        self.prev_left_pos = curr_left_pos
        self.prev_right_pos = curr_right_pos

        # Convert to meters
        d_left = d_left_rad * self.wheel_rad
        d_right = d_right_rad * self.wheel_rad

        # ---------- LOW-PASS FILTER ----------
        alpha = 0.5  # between 0 (smooth) and 1 (raw)
        d_left = alpha * d_left + (1 - alpha) * self.prev_d_left
        d_right = alpha * d_right + (1 - alpha) * self.prev_d_right
        self.prev_d_left = d_left
        self.prev_d_right = d_right
        # ------------------------------------

        # Differential drive model
        d_center = (d_left + d_right) / 2.0
        d_phi = (d_right - d_left) / self.wheel_sep

        # Ignore tiny random noise (helps when wheels not moving)
        if abs(d_center) < 1e-5 and abs(d_phi) < 1e-5:
            return

        # Update pose
        self.x += d_center * cos(self.th)
        self.y += d_center * sin(self.th)
        self.th += d_phi

        # Normalize heading (optional)
        self.th = (self.th + 3.14159) % (2 * 3.14159) - 3.14159

        # Publish Odometry
        self.publish_odom()

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (yaw only)
        q = self.euler_to_quaternion(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Reasonable covariance for EKF
        odom.pose.covariance = [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 99999, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, 0.03
        ]

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        return [qx, qy, qz, qw]

if __name__ == '__main__':
    try:
        OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
