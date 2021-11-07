"""lab3 controller."""
# Copyright Prof. Bradley Hayes <bradley.hayes@colorado.edu> 2021
# University of Colorado Boulder CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# Correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.16 # [m]
MAX_ROTATION_SPEED = 1.82 # [rad/2]



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

goal_arr = ((2.3, 2.1, 1.57), (.85, 3.75, 1.57), (0.54, 5.5, 1.57))
# goal_arr = ((1.5, 1, 3.14), (1.5, 1, 3.14))
goal_num = 0
goal_pos = goal_arr[goal_num]
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
target_pos = ('inf', 'inf')
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

BEARING_ERROR = 0.05 # approximately pi/6
POSITION_ERROR = 0.005 # 5cm
HEADING_ERROR = 0.05

while robot.step(timestep) != -1:
    #STEP 2: Calculate sources of error
    # Position error:
    rho = math.sqrt((goal_pos[0] - pose_x)**2 + (goal_pos[1] - pose_y)**2)
    # Bearing error:
    alpha = math.atan2((goal_pos[1] - pose_y), (goal_pos[0] - pose_x)) - pose_theta
    # Heading error:
    eta = goal_pos[2] - pose_theta
    pass   
    
    #STEP 2.4: Feedback Controller
    if abs(alpha) > BEARING_ERROR and abs(rho) > POSITION_ERROR:
        dXr = 0
        dTr = MAX_ROTATION_SPEED * (abs(alpha)/alpha) / 2
    elif abs(rho) > POSITION_ERROR:
        dXr = MAX_SPEED_MS/2
        dTr = 0
    elif abs(eta) > HEADING_ERROR:
        dXr = 0
        dTr = MAX_ROTATION_SPEED * (abs(eta)/eta) / 2
    pass
    
    #STEP 1.2: Inverse Kinematics Equations 
    veloL = dXr - dTr*AXLE_LENGTH/2 # Velocity in m/s
    veloR = dXr + dTr*AXLE_LENGTH/2
    pass
    
    # STEP 2.5: Compute wheel velocities (vL, vR)
    vL = veloL/MAX_SPEED_MS * MAX_SPEED
    vR = veloR/MAX_SPEED_MS * MAX_SPEED
    pass
    

    #STEP 2.7: Proportional velocities
    pass

    #STEP 2.6: Clamp wheel speeds
    if abs(vL) > MAX_SPEED:
        print(vL)
        vL = (vL/abs(vL)) * MAX_SPEED
    if abs(vR) > MAX_SPEED:
        print(vR)
        vR = (vR/abs(vR)) * MAX_SPEED
        
    pass

    if(abs(rho) < POSITION_ERROR and abs(eta) < HEADING_ERROR):
        goal_num = goal_num + 1
        if goal_num >= len(goal_arr):
            vL = 0
            vR = 0
        else:
            goal_pos = goal_arr[goal_num]
        
    # Odometry code. Don't change speeds (vL and vR) after this line
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28

    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
   

    