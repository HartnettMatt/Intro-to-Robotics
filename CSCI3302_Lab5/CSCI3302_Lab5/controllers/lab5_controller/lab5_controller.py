"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by posistion control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
#mode = 'manual' # Part 1.1: manual mode
mode = 'planner'
# mode = 'autonomous'




###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = (4.46793, 8.05674) # (Pose_X, Pose_Z) in meters
    end_w = (10.0, 7.0) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = [int(360*start_w[0]/12), int(360*start_w[1]/12)] # (x, y) in 360x360 map
    end = [int(360*end_w[0]/12), int(360*end_w[1]/12)] # (x, y) in 360x360 map                

    def find_min_d_in_Q(d, Q):
        minimum = d[Q[0][0]][Q[0][1]]
        minimum_p = [Q[0][0], Q[0][1]]
        for point in Q:
            if d[point[0]][point[1]] < minimum:
                minimum = d[point[0]][point[1]]
                minimum_p = [point[0], point[1]]
        return minimum_p

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
        # Dijkstra's attempt #1:
        # d = np.full((360, 360), np.inf, dtype=float)
        # prev = np.full((360,360), None, dtype=tuple)
        # d[start] = 0
        # Q = np.empty([1,2])

        # for x in range(360):
            # for y in range(360):
                # if(map[x][y] != 1):
                    # temp = [x,y]
                    # np.append(Q, temp) #issue Q not array of tuples

        # print(Q)
        # while len(Q) != 0:
            # u = np.where(Q == np.argmin(d), Q) #issue not returning a singple tuple
            # np.delete(Q, u)
            # print(u)
            # if u == end:
                # break
                
            
            # for i in range(-1,2):
                # for j in range(-1,2):
                    # v = (u[0] + i, u[1] + j) #issue V 1d array of many values not a tuple
                    # #print(v)
                    # if v in Q:
                        # if abs(i) == abs(j) and abs(i) == 1:
                            # Diagonal
                            # alt = math.sqrt(2)
                        # elif abs(i) != abs(j):
                            # Straight line
                            # alt = 1
                        # if alt < d[u[0]][u[1]]:
                            # d[v[0]][v[1]] = alt
                            # prev[v] = u
        # Out of while loop
        # s = np.array([], dtype=tuple)
        # u = end
        # if prev[u] is not None or u == start:
            # while u is not None:
                # print(u)
                # s = np.append(s, u)
                # u = prev[u[0]][u[1]]
                
        # Dijkstra's attempt #2:
        prev = [[[None, None] for j in range(360)] for i in range(360)]
        d = [[np.inf for j in range(360)] for i in range(360)]
        # Q = np.empty([360*360, 2])
        Q = []
        for x in range(360):
            for y in range(360):
                if map[x][y] != 1:
                    Q.append([x, y])
        d[start[0]][start[1]] = 0
        while len(Q) != 0:
        # Find minimum distance point in Q
            u = find_min_d_in_Q(d, Q)
            Q.remove(u)
            print(len(Q))
            
            for i in range(-1,2):
                for j in range(-1,2):
                    v = (u[0] + i, u[1] + j) #issue V 1d array of many values not a tuple
                    #print(v)
                    if v in Q:
                        if abs(i) == abs(j) and abs(i) == 1:
                            # Diagonal
                            alt = math.sqrt(2)
                        elif abs(i) != abs(j):
                            # Straight line
                            alt = 1
                        if alt < d[u[0]][u[1]]:
                            d[v[0]][v[1]] = alt
                            prev[v] = u
        # Out of while loop
        s = []
        u = end
        # Build the path array
        if prev[u[0]][u[1]] is not None or u == start:
            while u is not None:
                s.append([u[0], u[1]])
                u = prev[u[0]][u[1]]
        return s
        pass

    # Part 2.1: Load map (map.npy) from disk and visualize it
    map = np.load('map_full.npy')
    #plt.imshow(np.fliplr(map))
    #plt.show()

    # Part 2.2: Compute an approximation of the “configuration space”
    kernal_size = 15 # should provide 15 pixels of padding, which is equal to 0.5m
    kernal = np.ones((kernal_size, kernal_size))
    convoluted_map = convolve2d(map, kernal, mode='same')
    convoluted_map = 1*(convoluted_map >= 1)
    #plt.imshow(np.fliplr(convoluted_map))
    #plt.show()
    
    # Part 2.3 continuation: Call path_planner
    s = path_planner(convoluted_map, start, end)

    print(s)
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    waypoints = 12*(s/360.0)
    display.setColor(int(0xFF00FF))
    for tuple in s:
        # print(tuple)
        x = tuple[0]
        y = tuple[1]
        display.drawPixel(360-int(x*30),int(y*30))
        # print(i)
        #display.drawPixel(int(waypoints[i][0]), int(waypoints[i][1]))
    plt.imshow(np.fliplr(convoluted_map))
    plt.show()
    
    np.save('path.npy', waypoints)

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
map = np.zeros((360, 360)) # Replace None by a numpy 2D floating point array
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = np.load('path.npy')

state = 0 # use this to iterate through your path




while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
    
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            wx_int = int(360*wx/12)
            wy_int = int(360*wy/12)
            if(wx_int < 0):
                wx_int = 0
            if(wx_int > 359):
                wx_int = 359
            if(wy_int < 0):
                wy_int = 0
            if(wy_int > 359):
                wy_int = 359
            map[wx_int][wy_int] = map[wx_int][wy_int] + 0.005
            
            if(map[wx_int][wy_int] > 1.0):
                map[wx_int][wy_int] = 1.0
            g = int(map[wx_int][wy_int] * 255) # converting [0,1] to grayscale intensity [0,255]
            color = g*256**2+g*256+g
            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            display.setColor(color)
            display.drawPixel(360-int(wy*30),int(wx*30))

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            bool_map = map > 0.5
            int_map = np.multiply(bool_map, 1)
            np.save('map.npy', int_map)
            # plt.imshow(np.fliplr(int_map))
            # plt.show()
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            load_map = np.load("map.npy")
            print("Map loaded")
            map = load_map + map
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        # Part 3.2: Feedback controller
        #STEP 1: Calculate the error
        # position error
        rho = math.sqrt((waypoint[state][1] - pose_y)**2 + (waypoint[state][0] - pose_x)**2)
        # bearing error
        alpha = -(math.atan2(waypoint[state][1]-pose_y,waypoint[state][0]-pose_x) + pose_theta)


        #STEP 2: Controller
        BEARING_ERROR = 0.05
        HEADING_ERROR = 0.005
        MAX_ROTATION_SPEED = 1.75
        if abs(alpha) > BEARING_ERROR and abs(rho) > POSITION_ERROR:
            dX = 0
            dTheta = MAX_ROTATION_SPEED * (abs(alpha)/alpha) / 2
        elif abs(rho) > POSITION_ERROR:
            dX = MAX_SPEED_MS/2
            dTheta = 0
        elif state < len(waypoint):
            state = state + 1
        # elif abs(eta) > HEADING_ERROR:
            # dXr = 0
            # dTheta = MAX_ROTATION_SPEED * (abs(eta)/eta) / 2
        # dX = 0
        # dTheta = 0

        #STEP 3: Compute wheelspeeds
        vL = dX - dTheta*AXLE_LENGTH/2
        vR = dX + dTheta*AXLE_LENGTH/2

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        MAX_SPEED_MOD = 0.9
        if vL > MAX_SPEED*MAX_SPEED_MOD:
            vL = MAX_SPEED*MAX_SPEED_MOD
        if vR > MAX_SPEED*MAX_SPEED_MOD:
            vR = MAX_SPEED*MAX_SPEED_MOD
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)