"""state_machine controller."""

def sleep(duration):
    # Waits for duration seconds before returning
    global robot
    end_time = robot.getTime() + duration
    while robot.step(TIME_STEP) != -1 and robot.getTime() < end_time:
        pass

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

current_state = 0
# state 0 = forward 1
# state 1 = rotate 180
# state 2 = forward 2
# state 3 = rotate clockwise
# state 4 = forward 3
# state 5 = stop

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

# Professor said that an intensity of 80 is a good value for this lab
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Process sensor data here.
    if current_state == 0:
        if psValues[0] < 80:
            # go forward
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        elif psValues[0] >= 80:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 1
    elif current_state == 1:
        leftMotor.setVelocity(-3.14)
        rightMotor.setVelocity(3.14)
        sleep(1.569)
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        current_state = 2
    elif current_state == 2:
        if psValues[0] < 80:
            # go forward
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        elif psValues[0] >= 80:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 3
    elif current_state == 3:
        if psValues[5] < 80:
            # go forward
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(-3.0)
        elif psValues[5] >= 80:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 4
    elif current_state == 4:
        if psValues[5] >= 80:
            # go forward
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        elif psValues[5] < 80:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 5
    elif current_state == 5:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
