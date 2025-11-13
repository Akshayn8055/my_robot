#!/usr/bin/env python3
"""
ROS Control Hardware Interface for Raspberry Pi Robot
This script creates a hardware interface that:
1. Reads encoder data from GPIO pins
2. Calculates wheel positions and velocities
3. Receives velocity commands from diff_drive_controller
4. Converts velocity commands to PWM for motors
5. Publishes all data through ROS Control framework
"""
import rospy
import pigpio
import math
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Hardware Libraries
import board
import adafruit_mpu6050

# ======================
# PIN DEFINITIONS - Same as your current setup
# ======================
# Motor Control Pins (GPIO BCM numbering)
MOTOR_LEFT_PWM = 12     # PWM for left motor speed
MOTOR_LEFT_DIR1 = 20    # Direction control 1
MOTOR_LEFT_DIR2 = 21    # Direction control 2
MOTOR_RIGHT_PWM = 13    # PWM for right motor speed
MOTOR_RIGHT_DIR1 = 23   # Direction control 1
MOTOR_RIGHT_DIR2 = 24   # Direction control 2

# Encoder Pins (GPIO BCM numbering)
ENCODER_LEFT_A = 5      # Left encoder channel A
ENCODER_LEFT_B = 6      # Left encoder channel B
ENCODER_RIGHT_A = 26    # Right encoder channel A
ENCODER_RIGHT_B = 16    # Right encoder channel B

# ==================================
# ROBOT PHYSICAL PARAMETERS - MUST MATCH YOUR ROBOT
# ==================================
WHEEL_DIAMETER = 0.065          # meters (65mm wheels) [cite: 2195]
WHEEL_BASE = 0.20               # meters (20cm between wheels) [cite: 2197]
ENCODER_TICKS_PER_REV = 420     # Encoder resolution [cite: 2199]
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV [cite: 2200]

# PWM Configuration
PWM_FREQ = 5000                 # 5 kHz PWM frequency [cite: 2202]
PWM_RANGE = 255                 # 8-bit PWM (0-255) [cite: 2204]

# ==================================
# Velocity to PWM Conversion Factor
# ==================================
# Adjust this if robot moves too fast/slow
VELOCITY_TO_PWM_SCALE = 200.0   # Higher = more aggressive [cite: 2212]

# Maximum velocities (safety limits)
MAX_WHEEL_VELOCITY = 2.0        # rad/s (adjust based on your motors) [cite: 2214]

# =================
# GLOBAL VARIABLES
# =================
# Encoder counts (updated by interrupt handlers)
encoder_left_count = 0
encoder_right_count = 0

encoder_left_count_prev = 0
encoder_right_count_prev = 0

# Wheel positions (in radians)
left_wheel_position = 0.0
right_wheel_position = 0.0

# Wheel velocities (in rad/s)
left_wheel_velocity = 0.0
right_wheel_velocity = 0.0

# Velocity commands from controller (in rad/s)
left_wheel_velocity_cmd = 0.0
right_wheel_velocity_cmd = 0.0

# Timing
last_time = None
pi = None  # Global pigpio instance

# ====================================
# INTERRUPT SERVICE ROUTINES FOR ENCODERS
# ====================================
def encoder_left_callback(gpio, level, tick):
    """
    Interrupt handler for left encoder.
    Called every time encoder channel A transitions from LOW to HIGH.
    Increments or decrements count based on channel B state.
    """
    global encoder_left_count
    # Check channel B to determine direction
    if pi.read(ENCODER_LEFT_B):
        encoder_left_count += 1  # Forward
    else:
        encoder_left_count -= 1  # Backward

def encoder_right_callback(gpio, level, tick):
    """
    Interrupt handler for right encoder.
    Called every time encoder channel A transitions from LOW to HIGH.
    """
    global encoder_right_count
    # Check channel B to determine direction
    if pi.read(ENCODER_RIGHT_B):
        encoder_right_count -= 1  # Forward (reversed due to mounting)
    else:
        encoder_right_count += 1  # Backward

# ========================
# MOTOR CONTROL FUNCTIONS
# ========================
def set_motor_speed(motor_pwm_pin, motor_dir1_pin, motor_dir2_pin, speed):
    """
    Set motor speed and direction.
    Args:
      motor_pwm_pin: GPIO pin for PWM control
      motor_dir1_pin: GPIO pin for direction control 1
      motor_dir2_pin: GPIO pin for direction control 2
      speed: Motor speed from -255 to +255
             Positive = forward, Negative = backward
    """
    # Determine direction
    direction = speed >= 0
    pwm_value = int(abs(speed))

    # Clamp PWM value to valid range
    if pwm_value > PWM_RANGE:
        pwm_value = PWM_RANGE

    # Set direction pins
    if direction:
        pi.write(motor_dir1_pin, 1)
        pi.write(motor_dir2_pin, 0)
    else:
        pi.write(motor_dir1_pin, 0)
        pi.write(motor_dir2_pin, 1)
    
    # Set PWM duty cycle
    pi.set_PWM_dutycycle(motor_pwm_pin, pwm_value)

def velocity_to_pwm(velocity_rad_s):
    """
    Convert wheel velocity (rad/s) to PWM value (0-255).
    This function maps the desired wheel velocity to a PWM duty cycle.
    You may need to tune VELOCITY_TO_PWM_SCALE for your specific motors.
    Args:
      velocity_rad_s: Desired wheel velocity in radians per second
    Returns:
      PWM value (-255 to +255)
    """
    # Simple linear scaling
    pwm = velocity_rad_s * VELOCITY_TO_PWM_SCALE

    # Clamp to PWM range
    if pwm > PWM_RANGE:
        pwm = PWM_RANGE
    elif pwm < -PWM_RANGE:
        pwm = -PWM_RANGE
    
    return pwm

# =======================================
# HARDWARE INTERFACE CLASS
# =======================================
class PiHardwareInterface:
    """
    ROS Control Hardware Interface for Raspberry Pi Robot.
    This class implements the hardware interface layer that connects
    ROS Control to your robot's physical hardware (motors and encoders).
    """
    def __init__(self):
        """Initialize the hardware interface."""
        rospy.loginfo("Initializing Pi Hardware Interface...")

        # Initialize timing
        global last_time
        last_time = time.time()

        # Initialize pigpio
        global pi
        pi = pigpio.pi()
        if not pi.connected:
            rospy.logerr("Failed to connect to pigpio daemon!")
            rospy.logerr("Make sure pigpiod is running: sudo systemctl start pigpiod")
            raise Exception("pigpio connection failed")
        
        rospy.loginfo("Connected to pigpio daemon")

        # Setup motor pins
        self._setup_motor_pins()
        
        # Setup encoder pins
        self._setup_encoder_pins()

        # Initialize IMU
        self._setup_imu()

        # Create ROS publishers
        self.joint_state_pub = rospy.Publisher(
            'joint_states',
            JointState,
            queue_size=10
        )
        self.imu_pub = rospy.Publisher(
            'imu/data_raw',
            Imu,
            queue_size=10
        )

        rospy.loginfo("Pi Hardware Interface initialized successfully!")

    def _setup_motor_pins(self):
        """Configure GPIO pins for motor control."""
        rospy.loginfo("Setting up motor control pins...")
        # Set motor direction pins as outputs
        for pin in [MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2]:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.write(pin, 0)  # Initialize to LOW
        
        # Set motor PWM pins
        for pin in [MOTOR_LEFT_PWM, MOTOR_RIGHT_PWM]:
            pi.set_PWM_frequency(pin, PWM_FREQ)
            pi.set_PWM_range(pin, PWM_RANGE)
            pi.set_PWM_dutycycle(pin, 0)  # Start with motors off
        
        rospy.loginfo("Motor pins configured")

    def _setup_encoder_pins(self):
        """Configure GPIO pins for encoder reading and attach interrupts."""
        rospy.loginfo("Setting up encoder pins...")
        # Set encoder pins as inputs with pull-up resistors
        for pin in [ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B]:
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)
        
        # Attach interrupt handlers to encoder A channels
        # These fire on RISING edge (LOW to HIGH transition)
        pi.callback(ENCODER_LEFT_A, pigpio.RISING_EDGE, encoder_left_callback)
        pi.callback(ENCODER_RIGHT_A, pigpio.RISING_EDGE, encoder_right_callback)
        
        rospy.loginfo("Encoder interrupts attached")

    def _setup_imu(self):
        """Initialize the IMU sensor."""
        try:
            i2c = board.I2C()  # <-- FIX 1: Corrected variable assignment
            self.mpu = adafruit_mpu6050.MPU6050(i2c)
            rospy.loginfo("MPU6050 IMU Initialized")
            self.imu_available = True # <-- FIX 2: Corrected variable name
        except Exception as e: # <-- FIX 3: Corrected exception syntax
            rospy.logwarn(f"Failed to initialize IMU: {e}")
            rospy.logwarn("Continuing without IMU data")
            self.imu_available = False

    def read(self):
        """
        Read current state from hardware.
        This method:
        1. Reads encoder counts
        2. Calculates wheel positions (radians)
        3. Calculates wheel velocities (rad/s)
        4. Updates global state variables
        Called by the main control loop before each controller update.
        """
        global encoder_left_count, encoder_right_count
        global encoder_left_count_prev, encoder_right_count_prev
        global left_wheel_position, right_wheel_position
        global left_wheel_velocity, right_wheel_velocity
        global last_time

        # Get current time
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Prevent division by zero
        if dt == 0:
            dt = 0.001

        # Read current encoder counts (from interrupt handlers)
        current_left_count = encoder_left_count
        current_right_count = encoder_right_count

        # Calculate change in encoder counts
        delta_left = current_left_count - encoder_left_count_prev
        delta_right = current_right_count - encoder_right_count_prev

        # Update previous counts
        encoder_left_count_prev = current_left_count
        encoder_right_count_prev = current_right_count

        # Convert encoder ticks to wheel position change (radians)
        # Position = (ticks / ticks_per_rev) * 2π
        delta_left_rad = (delta_left / ENCODER_TICKS_PER_REV) * 2.0 * math.pi
        delta_right_rad = (delta_right / ENCODER_TICKS_PER_REV) * 2.0 * math.pi

        # Update wheel positions (integrate)
        left_wheel_position += delta_left_rad
        right_wheel_position += delta_right_rad

        # Calculate wheel velocities (rad/s)
        left_wheel_velocity = delta_left_rad / dt
        right_wheel_velocity = delta_right_rad / dt

    def write(self):
        """
        Write commands to hardware.
        This method:
        1. Takes velocity commands from controller
        2. Converts to PWM values
        3. Sends to motor driver
        Called by the main control loop after each controller update.
        """
        global left_wheel_velocity_cmd, right_wheel_velocity_cmd

        # Get velocity commands (set by controller)
        left_vel = left_wheel_velocity_cmd
        right_vel = right_wheel_velocity_cmd

        # Apply safety limits
        left_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, left_vel))
        right_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, right_vel))

        # Convert velocities to PWM
        left_pwm = velocity_to_pwm(left_vel)
        right_pwm = velocity_to_pwm(right_vel)

        # Send to motors
        set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, left_pwm)
        set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, right_pwm)

    def publish_joint_states(self):
        """
        Publish current joint states to ROS.
        Joint states include:
        - Joint names (left_wheel_joint, right_wheel_joint)
        - Positions (radians)
        - Velocities (rad/s)
        - Efforts (not used, set to 0)
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [left_wheel_position, right_wheel_position]
        msg.velocity = [left_wheel_velocity, right_wheel_velocity]
        msg.effort = [0.0, 0.0]  # We don't measure motor current
        
        self.joint_state_pub.publish(msg)

    def publish_imu(self):
        """Publish IMU data if available."""
        if not self.imu_available: # <-- FIX 4: Corrected variable name
            return

        try:
            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_link"
            
            # Read accelerometer (m/s^2)
            accel = self.mpu.acceleration
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

            # Read gyroscope (rad/s)
            gyro = self.mpu.gyro
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            # No magnetometer data (disabled)
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 0.0  # Note: A real IMU would provide orientation

            self.imu_pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"IMU read error: {e}")

    def shutdown(self):
        """
        Cleanup when node shuts down.
        Stops motors and releases GPIO resources.
        """
        rospy.loginfo("Shutting down hardware interface...")
        # Stop motors
        set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, 0)
        set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, 0)
        
        # Release pigpio resources
        if pi:
            pi.stop()
        
        rospy.loginfo("Hardware Interface shutdown complete")

# ============================
# VELOCITY COMMAND CALLBACKS
# ============================
def left_wheel_vel_callback(msg):
    """
    Callback for left wheel velocity commands from controller.
    The diff_drive_controller publishes to these topics:
    - /left_wheel_velocity_controller/command
    - /right_wheel_velocity_controller/command
    """
    global left_wheel_velocity_cmd
    left_wheel_velocity_cmd = msg.data

def right_wheel_vel_callback(msg):
    """Callback for right wheel velocity commands from controller."""
    global right_wheel_velocity_cmd
    right_wheel_velocity_cmd = msg.data

# =================
# MAIN FUNCTION
# =================
def main():
    """Main control loop."""
    # Initialize ROS node
    rospy.init_node('pi_hardware_interface')
    rospy.loginfo("Starting Pi Hardware Interface Node...")

    # Create hardware interface
    hw_interface = PiHardwareInterface()
    
    # Register shutdown hook
    rospy.on_shutdown(hw_interface.shutdown)

    # Subscribe to velocity commands from controller
    # NOTE: These topic names are set by the controller configuration
    rospy.Subscriber(
        '/left_wheel_velocity_controller/command',
        Float64,
        left_wheel_vel_callback
    )
    rospy.Subscriber(
        '/right_wheel_velocity_controller/command',
        Float64,
        right_wheel_vel_callback
    )

    # Control loop rate (Hz)
    rate = rospy.Rate(50)  # 50 Hz - adjust if needed [cite: 2531]
    rospy.loginfo("Hardware Interface running at 50 Hz")
    rospy.loginfo("Waiting for velocity commands...")

    # Main control loop
    try:
        while not rospy.is_shutdown():
            # Read from hardware (encoders)
            hw_interface.read()
            
            # Write to hardware (motors)
            hw_interface.write()
            
            # Publish joint states
            hw_interface.publish_joint_states()
            
            # Publish IMU data
            hw_interface.publish_imu()

            # Sleep to maintain loop rate
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        # Shutdown is handled by the rospy.on_shutdown hook
        pass

# =================
# ENTRY POINT
# =================
if __name__ == '__main__': # <-- FIX 5: Corrected main entry point syntax
    main()

