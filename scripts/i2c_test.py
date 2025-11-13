import smbus2

# MPU6050 Registers
I2C_BUS = 1
DEVICE_ADDRESS = 0x68
WHO_AM_I_REG = 0x75

try:
    # Open a connection to the I2C bus
    bus = smbus2.SMBus(I2C_BUS)

    # Read the WHO_AM_I register
    device_id = bus.read_byte_data(DEVICE_ADDRESS, WHO_AM_I_REG)

    print(f"Successfully connected to device at address 0x{DEVICE_ADDRESS:02x}")
    print(f"Device ID (WHO_AM_I): 0x{device_id:02x}")

    if device_id == 0x68:
        print("This is a genuine MPU6050 sensor.")
    else:
        print("Warning: Device ID is not the expected 0x68. This may be a clone or a different sensor.")

    bus.close()

except Exception as e:
    print(f"Error: Could not connect to the MPU6050.")
    print(f"Details: {e}")
