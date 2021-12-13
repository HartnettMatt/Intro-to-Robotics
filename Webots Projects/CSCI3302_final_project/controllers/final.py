"""final_controller controller."""
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Keyboard, Motor, DistanceSensor

LIDAR_MAX_RANGE = 30
VERTICAL_THRUST = 68.5
VERTICAL_OFFSET = .6
VERTICAL_P = 3.0
ROLL_P = 50.0
PITCH_P = 30.0

MAP_WIDTH = 150
MAP_HEIGHT = 50

LIDAR_SENSOR_MAX_RANGE = 3 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

goal_altitude = 0.5 # meters

# mode = "manual"
mode = "path_following"

x = 3
y = -3
z = 2
th = 0
path_index = 0
path = [[x, y, z, th], [0, 0, 1, math.pi]]
BEARING_ERROR = 0.10 # approximately pi/3
POSITION_ERROR = 0.05 # 5cm
HEADING_ERROR = 0.1

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Enable sensors
camera = robot.getDevice("camera")
camera.enable(SIM_TIMESTEP)
imu = robot.getDevice("inertial unit")
imu.enable(SIM_TIMESTEP)
gps = robot.getDevice("gps")
gps.enable(1)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)
gyro = robot.getDevice("gyro")
gyro.enable(SIM_TIMESTEP)
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

# Get motors
fl_motor = robot.getDevice("front left propeller")
fr_motor = robot.getDevice("front right propeller")
rl_motor = robot.getDevice("rear left propeller")
rr_motor = robot.getDevice("rear right propeller")

# Enable keyboard controls (for manual)
keyboard = Keyboard();
keyboard.enable(SIM_TIMESTEP)

motors = [fl_motor, fr_motor, rl_motor, rr_motor]

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

for motor in motors:
    motor.setPosition(math.inf)
    motor.setVelocity(1)

lidar_sensor_readings = []
lidar_offsets = np.linspace(-0.5*LIDAR_ANGLE_RANGE, 0.5*LIDAR_ANGLE_RANGE, LIDAR_ANGLE_BINS)
lidar_map = np.full((MAP_WIDTH, MAP_WIDTH, MAP_HEIGHT), -1.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
    RollPitchYaw = imu.getRollPitchYaw()
    roll = RollPitchYaw[0] + math.pi / 2
    pitch = RollPitchYaw[1]
    yaw = RollPitchYaw[2]
    gps_values = gps.getValues()
    gyros = gyro.getValues()
    roll_acceleration = gyros[0]
    pitch_acceleration = gyros[1]
    lidar_sensor_readings = lidar.getRangeImage()
    pose_x = gps_values[0]
    pose_z = gps_values[1]
    pose_y = gps_values[2]
    pose_theta = -1*yaw

    # Pose of the robot in terms of the lidar map
    pixel_pose_x = (int)((pose_x+7.5)*MAP_WIDTH/15)
    pixel_pose_y = (int)((pose_y+7.5)*MAP_WIDTH/15)
    pixel_pose_z = (int)(pose_z*MAP_HEIGHT/5)

    # Process sensor data here.

    # Map LIDAR data
    transform = [[math.sin(pose_theta), -1*math.cos(pose_theta), pose_x],
                 [math.cos(pose_theta), math.sin(pose_theta), pose_y],
                 [0,0,1]]
    lidar_world_x = []
    lidar_world_y = []
    for i in range(len(lidar_sensor_readings)):
        if(lidar_sensor_readings[i] < LIDAR_SENSOR_MAX_RANGE):
            lidar_robot_x = lidar_sensor_readings[i]*math.cos(lidar_offsets[i])
            lidar_robot_y = lidar_sensor_readings[i]*math.sin(lidar_offsets[i])
            lidar_robot = [[lidar_robot_x],
                           [lidar_robot_y],
                           [1]]
            lidar_world = np.matmul(transform, lidar_robot)
            lidar_world_x.append(lidar_world[0][0])
            lidar_world_y.append(lidar_world[1][0])


    for i in range(len(lidar_world_x)):
        pixel_lidar_world_x = (int)((lidar_world_x[i]+7.5)*MAP_WIDTH/15)
        pixel_lidar_world_y = (int)((lidar_world_y[i]+7.5)*MAP_WIDTH/15)
        lidar_map[pixel_lidar_world_x][pixel_lidar_world_y][pixel_pose_z] = 1
        delX = lidar_world_x[i] - pose_x
        delY = lidar_world_y[i] - pose_y
        dX = delX/n
        dY = delY/n
        for j in range(n):
            x_coord = pixel_pose_x + dX*n
            y_coord = pixel_pose_y + dY*n
            lidar_map[x_coord][y_coord][pixel_pose_z] = 0

        display.setColor(0xFFFFFF)
        display.drawLine(pixel_pose_x, pixel_pose_y, pixel_lidar_world_x, pixel_lidar_world_y)
        display.setColor(0x0000FF)
        display.drawPixel(pixel_lidar_world_x, pixel_lidar_world_y)


    display.setColor(0xFF0000)
    display.drawPixel(pixel_pose_x, pixel_pose_y)

    # Handle movement controls
    roll_disturbance = 0
    pitch_disturbance = 0
    yaw_disturbance = 0

    if mode == "manual":
        key = keyboard.getKey()
        if key == Keyboard.UP:
            pitch_disturbance = 2.0
        elif key == Keyboard.DOWN:
            pitch_disturbance = -2.0
        elif key == Keyboard.RIGHT:
            yaw_disturbance = 1.3
        elif key == Keyboard.LEFT:
            yaw_disturbance = -1.3
        elif key == keyboard.SHIFT + Keyboard.UP:
            goal_altitude = goal_altitude + 0.01
        elif key == keyboard.SHIFT + Keyboard.DOWN:
            goal_altitude = goal_altitude - 0.01
    elif mode == "path_following":
        print("gps values: " + str(gps_values) + " yaw: " + str(yaw))
        goal_altitude = path[path_index][2]
        # Position error:
        rho = math.sqrt((path[path_index][0] - pose_x)**2 + (path[path_index][1] - pose_y)**2)
        # Bearing error:
        alpha = math.atan2((path[path_index][1] - pose_y), (path[path_index][0] - pose_x)) - pose_theta
        alpha1 = math.atan2((pose_y - path[path_index][1]), (pose_x - path[path_index][0])) - pose_theta
        # Heading error:
        eta = path[path_index][3] - pose_theta
        print("alpha: " + str(alpha) + " alpha1: " + str(alpha1))
        # Turn to face destination
        if abs(alpha) > BEARING_ERROR and abs(rho) > POSITION_ERROR:
            yaw_disturbance = 0.1 * (alpha/abs(alpha))
        # Move forward until you get there
        elif abs(rho) > POSITION_ERROR:
            pitch_disturbance = 0.5
        # Turn the final direction
        elif abs(eta) > HEADING_ERROR:
            yaw_disturbance = 0.1 * (eta/abs(eta))

    roll_input = ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
    pitch_input = PITCH_P * clamp(pitch, -1, 1) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_altitude_difference = clamp(goal_altitude - pose_z + VERTICAL_OFFSET, -1, 1)
    vertical_input = VERTICAL_P * (clamped_altitude_difference**3)

    fl_motor_in = VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    fr_motor_in = -1*(VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input)
    rl_motor_in = -1*(VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input)
    rr_motor_in = VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input

    fl_motor.setVelocity(fl_motor_in)
    fr_motor.setVelocity(fr_motor_in)
    rl_motor.setVelocity(rl_motor_in)
    rr_motor.setVelocity(rr_motor_in)


    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
