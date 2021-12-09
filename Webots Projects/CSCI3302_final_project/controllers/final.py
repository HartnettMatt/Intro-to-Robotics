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

goal_altitude = 1.5 # meters

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(SIM_TIMESTEP)
imu = robot.getDevice("inertial unit")
imu.enable(SIM_TIMESTEP)
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)
gyro = robot.getDevice("gyro")
gyro.enable(SIM_TIMESTEP)
fl_motor = robot.getDevice("front left propeller")
fr_motor = robot.getDevice("front right propeller")
rl_motor = robot.getDevice("rear left propeller")
rr_motor = robot.getDevice("rear right propeller")
keyboard = Keyboard();
keyboard.enable(SIM_TIMESTEP)

motors = [fl_motor, fr_motor, rl_motor, rr_motor]

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

for motor in motors:
    motor.setPosition(math.inf)
    motor.setVelocity(1)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
    RollPitchYaw = imu.getRollPitchYaw()
    roll = RollPitchYaw[0] + math.pi / 2
    pitch = RollPitchYaw[1]
    yaw = RollPitchYaw[2]
    altitude = gps.getValues()[1]
    gyros = gyro.getValues()
    roll_acceleration = gyros[0]
    pitch_acceleration = gyros[1]

    # Process sensor data here.
    roll_disturbance = 0
    pitch_disturbance = 0
    yaw_disturbance = 0
    key = keyboard.getKey()
    if key == Keyboard.UP:
        pitch_disturbance = 2.0
    elif key == Keyboard.DOWN:
        pitch_disturbance = -2.0
    elif key == Keyboard.RIGHT:
        yaw_disturbance = 1.3
    elif key == Keyboard.LEFT:
        yaw_disturbance = -1.3

    roll_input = ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
    pitch_input = PITCH_P * clamp(pitch, -1, 1) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_altitude_difference = clamp(goal_altitude - altitude + VERTICAL_OFFSET, -1, 1)
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
