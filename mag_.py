# coding=utf-8
import smbus  # import SMBus module of I2C
from time import sleep  # import sleep
import math

# some MPU6050 Registers and their Address
Register_A = 0  # Address of Configuration register A
Register_B = 0x01  # Address of configuration register B
Register_mode = 0x02  # Address of mode register

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value
calibrated_values = [0, 0, 0]

scaler_flag = False

#
def Magnetometer_Init():
    # write to Configuration Register A
    bus.write_byte_data(Device_Address, Register_A, 0x70)

    # Write to Configuration Register B for gain
    bus.write_byte_data(Device_Address, Register_B, 0xa0)

    # Write to mode Register for selecting mode
    bus.write_byte_data(Device_Address, Register_mode, 0)


def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from module
    if (value > 32768):
        value = value - 65536
    return value


def getHeading():
    return [read_raw_data(X_axis_H)
        , read_raw_data(Y_axis_H)
        , read_raw_data(Z_axis_H)]


def transformation(H):
    uncalibrated_values = H
    calib_matrix = [
        [0.083832, 0.001101, -0.010638],
        [0.001101, 0.096019, -0.018923],
        [-0.010638, -0.018923, 0.113036]
    ]
    bias = [
        11.008123,
        -22.149114,
        -1.271789
    ]
    for i in range(0, 3): uncalibrated_values[i] = uncalibrated_values[i] - bias[i]
    result = [0.0,0.0,0.0]
    for i in range(0, 3):
        for j in range(0, 3):
            result[i] += calib_matrix[i][j] * uncalibrated_values[j]
    calibrated_values = result
    return calibrated_values


def vector_length_stabalization(H, scaler_flag):
    calibrated_values = H
    normal_vector_length = None
    if not scaler_flag:
        calibrated_values = transformation(H)
        normal_vector_length = math.sqrt(calibrated_values[0] ** 2 + calibrated_values[1] ** 2 + calibrated_values[2] ** 2)
        scaler_flag = True
    # calculate the current scaler
    scaler = normal_vector_length / math.sqrt(calibrated_values[0] **2+ calibrated_values[1]**2 +calibrated_values[2] **2)
    # apply the current scaler to the calibrated coordinates(global array calibrated_values)
    calibrated_values[0] = calibrated_values[0] * scaler
    calibrated_values[1] = calibrated_values[1] * scaler
    calibrated_values[2] = calibrated_values[2] * scaler
    return calibrated_values
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e  # HMC5883L magnetometer device address

Magnetometer_Init()  # initialize HMC5883L magnetometer
scaler_flag = False

while True:

    # Read Accelerometer raw value
    H = getHeading()
    #H=[-25.18,-51.82,-8.57]
    # calibration
    H = vector_length_stabalization(H,scaler_flag)#vector_length_stabalization(H[0], H[1], H[2], scaler_flag)
    heading = float(math.atan2(H[1], H[0])) + declination
    declinationAngle = 0.22
    heading += declinationAngle
    # Due to declination check for >360 degree
    # // Correct for when signs are reversed.
    if (heading < 0):
        heading += 2 * pi
    #  // Check for wrap due to addition of declination.
    if (heading > 2 * pi):
        heading -= 2 * pi
    # convert into angle
    heading_angle = int(heading * 180 / pi)
    #print(heading_angle)
    print("Heading Angle = %d°" % heading_angle)
    print("H",H)
    sleep(0.1)
# coding=utf-8
import smbus  # import SMBus module of I2C
from time import sleep  # import sleep
import math

# some MPU6050 Registers and their Address
Register_A = 0  # Address of Configuration register A
Register_B = 0x01  # Address of configuration register B
Register_mode = 0x02  # Address of mode register

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value
calibrated_values = [0, 0, 0]

scaler_flag = False

#
def Magnetometer_Init():
    # write to Configuration Register A
    bus.write_byte_data(Device_Address, Register_A, 0x70)

    # Write to Configuration Register B for gain
    bus.write_byte_data(Device_Address, Register_B, 0xa0)

    # Write to mode Register for selecting mode
    bus.write_byte_data(Device_Address, Register_mode, 0)


def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from module
    if (value > 32768):
        value = value - 65536
    return value


def getHeading():
    return [read_raw_data(X_axis_H)
        , read_raw_data(Y_axis_H)
        , read_raw_data(Z_axis_H)]


def transformation(H):
    uncalibrated_values = H
    calib_matrix = [
        [0.083832, 0.001101, -0.010638],
        [0.001101, 0.096019, -0.018923],
        [-0.010638, -0.018923, 0.113036]
    ]
    bias = [
        11.008123,
        -22.149114,
        -1.271789
    ]
    for i in range(0, 3): uncalibrated_values[i] = uncalibrated_values[i] - bias[i]
    result = [0.0,0.0,0.0]
    for i in range(0, 3):
        for j in range(0, 3):
            result[i] += calib_matrix[i][j] * uncalibrated_values[j]
    calibrated_values = result
    return calibrated_values


def vector_length_stabalization(H, scaler_flag):
    calibrated_values = H
    normal_vector_length = None
    if not scaler_flag:
        calibrated_values = transformation(H)
        normal_vector_length = math.sqrt(calibrated_values[0] ** 2 + calibrated_values[1] ** 2 + calibrated_values[2] ** 2)
        scaler_flag = True
    # calculate the current scaler
    scaler = normal_vector_length / math.sqrt(calibrated_values[0] **2+ calibrated_values[1]**2 +calibrated_values[2] **2)
    # apply the current scaler to the calibrated coordinates(global array calibrated_values)
    calibrated_values[0] = calibrated_values[0] * scaler
    calibrated_values[1] = calibrated_values[1] * scaler
    calibrated_values[2] = calibrated_values[2] * scaler
    return calibrated_values
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e  # HMC5883L magnetometer device address

Magnetometer_Init()  # initialize HMC5883L magnetometer
scaler_flag = False

while True:

    # Read Accelerometer raw value
    H = getHeading()
    #H=[-25.18,-51.82,-8.57]
    # calibration
    H = vector_length_stabalization(H,scaler_flag)#vector_length_stabalization(H[0], H[1], H[2], scaler_flag)
    heading = float(math.atan2(H[1], H[0])) + declination
    declinationAngle = 0.22
    heading += declinationAngle
    # Due to declination check for >360 degree
    # // Correct for when signs are reversed.
    if (heading < 0):
        heading += 2 * pi
    #  // Check for wrap due to addition of declination.
    if (heading > 2 * pi):
        heading -= 2 * pi
    # convert into angle
    heading_angle = int(heading * 180 / pi)
    #print(heading_angle)
    print("Heading Angle = %d°" % heading_angle)
    print("H",H)
    sleep(0.1)
