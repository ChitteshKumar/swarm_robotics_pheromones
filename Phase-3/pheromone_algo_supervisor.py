"""food_detection_avoid_obstacles controller."""
import numpy as np
from controller import Motor, Field,DistanceSensor, LED, Node, Camera, Emitter, Receiver, Supervisor
import sys
import cv2  # OpenCV for image processing
import math
import json
import matplotlib.pyplot as plt 

class InitializeRobot():
    def  __init__(self):
        # create the Robot instance.
        # robot = Robot()
        self.supervisor = Supervisor()

        #time step of the world
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        #Distance Sensor in the robot
        DISTANCE_SENSORS_NUMBER = 8
        self.distance_sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        self.distance_sensors =  [None] * DISTANCE_SENSORS_NUMBER
        self.distance_sensors_values = [0.0] * DISTANCE_SENSORS_NUMBER  # Initialize distance_sensors_values 
        for i, name in enumerate(self.distance_sensor_names):
            self.distance_sensors[i] = self.supervisor.getDevice(name)
            self.distance_sensors[i].enable(self.timestep)

        #Ground Sensor in the device
        GROUND_SENSORS_NUMBER = 3
        ground_sensor_names = ["gs0", "gs1", "gs2"]
        # Silently initialize the ground sensors if they exist
        self.ground_sensors = [None] * GROUND_SENSORS_NUMBER
        self.ground_sensors_values = [0.0] * GROUND_SENSORS_NUMBER

        devices_number = self.supervisor.getNumberOfDevices()
        for i in range(devices_number):
            dtag = self.supervisor.getDeviceByIndex(i)
            dname = dtag.getName()
            dtype = dtag.getNodeType()
            if dtype == Node.DISTANCE_SENSOR and len(dname) == 3 and dname[0] == 'g' and dname[1] == 's':
                id = int(dname[2])
                if 0 <= id < GROUND_SENSORS_NUMBER:
                    self.ground_sensors[id] = self.supervisor.getDevice(ground_sensor_names[id])
                    self.ground_sensors[id].enable(self.timestep)
        
        #Led in the device
        self.LEDS_NUMBER = 10
        self.led_names = ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"]
        self.leds = [None] * self.LEDS_NUMBER
        self.leds_values = [False] * self.LEDS_NUMBER  # Initialize leds_values
        for i, name in enumerate(self.led_names):
            self.leds[i] = self.supervisor.getDevice(name)
        
        #sensor for camera
        self.camera = self.supervisor.getDevice("camera")
        self.camera.enable(self.timestep)
        
        # Define emitter and receiver
        self.emitter = self.supervisor.getDevice("emitter")
        receiver = self.supervisor.getDevice("receiver")
        receiver.enable(self.timestep)
        
        #Initialising the motors 
        self.left_motor = self.supervisor.getDevice("left wheel motor")
        self.right_motor = self.supervisor.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        #About the robot
        # MAX_SPEED = 6.28 
        self.MAX_SPEED = 3.14
        self.speeds = [0.0, 0.0]
        self.offsets = [0.5 * self.MAX_SPEED, 0.5 * self.MAX_SPEED]
        self.coefficients = [[0.942, -0.22], [0.63, -0.1], [0.5, -0.06],  [-0.06, -0.06], [-0.06, -0.06], [-0.06, 0.5], [-0.19, 0.63], [-0.13, 0.942]]
        self.counter = 0
    
    
    def step(self):
        if self.supervisor.step(self.timestep) == -1:
            self.supervisor.cleanup()
            sys.exit(0)
    
    def passive_wait(self, sec):
        start_time = self.supervisor.getTime()
        while start_time + sec > self.supervisor.getTime():
            self.step()
    
    def reset_actuator_values(self):
        for i in range(2):
            self.speeds[i] = 0.0
        for i in range(self.LEDS_NUMBER):
            # leds[i].setValue(False)
            self.leds[i].set(0)
    
    def get_sensor_input(self):
        for i, sensor in enumerate(self.distance_sensors):
            self.distance_sensors_values[i] = sensor.getValue()
            # Scale the data to have a value between 0.0 and 1.0
            # 1.0 representing something to avoid, 0.0 representing nothing to avoid
            # distance_sensors_values[i] /= 4096
        
        for i, sensor in enumerate(self.ground_sensors):
            if sensor:
                self.ground_sensors_values[i] = sensor.getValue()

    def set_actuators(self):
        for i, led in enumerate(self.leds):
            if self.leds_values[i]:
                led.set(1)
            else:
                led.set(0)
        self.left_motor.setVelocity(self.speeds[0])
        self.right_motor.setVelocity(self.speeds[1])
    
    def blink_leds(self):
        # global counter
        self.counter += 1
        self.leds_values[(self.counter // 10) % self.LEDS_NUMBER] = True
    
    def go_forward(self):
        self.passive_wait(0.2)
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
        
    
    def go_backwards(self):
        self.passive_wait(0.2)
        self.left_motor.setVelocity(-self.MAX_SPEED)
        self.right_motor.setVelocity(-self.MAX_SPEED)
        
    
    def turn_left(self):
        self.passive_wait(0.2)
        self.left_motor.setVelocity(self.MAX_SPEED * 0.5) # slower velocity for turning 
        self.right_motor.setVelocity(self.MAX_SPEED)
        
        
    def turn_right(self):
        self.passive_wait(0.2)
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED * 0.5 )

    def get_camera_input(self):
        global number_pixels
        image = self.camera.getImage()
        # Assuming the camera resolution is 128x128 pixels
        width, height = self.camera.getWidth(), self.camera.getHeight()
        frame = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        # Convert to RGB color space for easier color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Define the range of red color in HSV
        lower_red = np.array([0, 150, 150])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Check if there are any red pixels in the image
        is_red = np.any(mask)
        number_pixels = np.sum(mask)
        # Print some debug information
        # print("Red detection:", is_red)
        print("Number of red pixels:", number_pixels)
        return is_red
    
    def cliff_detected(self):
         for i in range(self.GROUND_SENSORS_NUMBER):
             if self.ground_sensors[i] and self.ground_sensors[i] < 100.0:
                 return True
         return False
    
    def wall_detected(self):
        # Check if any distance sensor reading indicates a wall
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            if self.distance_sensors_values[i] > 80.0:
                return True  # Wall detected
        return False  # No wall detected

class RobotState:
    SEARCHING = 1
    HOMING = 2
    IDLE = 3

class PIDController:
    def __init__(self, start_, goal_, kp, ki, kd, theta_ ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.goal = goal_
        self.current = start_
        self.theta = theta_
        self.E = 0
        self.old_e = 0
        self.desiredV = 1.0

    def update(self):
        #difference in x and y 
        # dx = self.goal[0] - abs(self.current[0])
        # dy = self.goal[1] - abs(self.current[1])
        dx = self.goal[0] - self.current[0]
        dy = self.goal[1] - self.current[1]

        angle_to_start = math.atan2(dy, dx)
        # print("angle to start", angle_to_start)

        # alpha = angle_to_start - self.current.theta
        alpha = angle_to_start - np.radians(90)
        # print("aplha: ", alpha)
        e = math.atan2(np.sin(alpha), np.cos(alpha))

        ep = e
        ei = self.E + e
        ed  = e - self.old_e 
        output =  self.kp * ep+ self.ki * ei + self.kd * ed
        output = math.atan2(np.sin(output), np.cos(output))

        self.E = self.E + e
        self.old_e = e
        v = self.desiredV

        return v, output       

class PheromoneAlgorithm():

    def __init__(self, robot) -> None:
        #define pheromone properties
        self.intensity = 1.0
        self.distances = {}

        self.pheromone_grids = {}
        self.global_pheromone_grid = [[0.0]*5 for _ in range(5)]
        self.robot_positions = {}

        self.evaporation_rate = 0.1
        self.evaporation_interval = 2.0 # in seconds

        self.robot = robot
        # self.controller = Controller(robot)
        self.controller = None
        self.current_sensed_pheromone = 0
        
    def setController(self, controller):
        self.controller = controller

    def get_robot_position(self):
        robot_node = robot.supervisor.getSelf()
        position = robot_node.getPosition()
        return position
    
    def has_moved(self, robot_name, old_pos, new_pos, threshold= 0.0015):
        if old_pos is None:
            return True
        distance = np.sqrt(
            ( new_pos[0] - old_pos[0] )**2 + 
            ( new_pos[1]- old_pos[1])**2 
            )
        
        if robot_name not in self.distances:
            self.distances[robot_name] = []
        
        self.distances[robot_name].append(distance)
        # print(f"distance from the previous pos {robot_name}: ", distance)
        return distance >= threshold
    
    def getCoordinates(self):
        coordinate = self.get_robot_position()
        return coordinate
    
    #STEP-1
    # pheromone_grid = [[0.0]*10 for _ in range(10)]
   
    # def deposit_pheromone(self, robot_name, robot_pos):
    #     global global_pheromone_grid

    #     if robot_name not in self.pheromone_grids:
    #         self.pheromone_grids[robot_name] = [[0.0]*5 for _ in range(5)]
    
    #     # Store current and previous locations
    #     if robot_name not in self.robot_positions:  # Initialize storage for the robot
    #         self.robot_positions[robot_name] = {'current_pos': robot_pos, 'previous_pos': None}
    #         print(f"POSITION OF THE ROBOT {robot_name} : {self.robot_positions}")
    #     else:
    #         self.robot_positions[robot_name]['previous_pos'] = self.robot_positions[robot_name]['current_pos']  # Shift previous to old
    #         self.robot_positions[robot_name]['current_pos'] = robot_pos  # Update current
    #         print(f"POSITION OF THE ROBOT {robot_name} : {self.robot_positions}")

    #     distance_moved = self.has_moved(robot_name, self.robot_positions[robot_name]['previous_pos'], self.robot_positions[robot_name]['current_pos'])
    #     if distance_moved:
    #         x = abs(int(robot_pos[0]*10))
    #         y = abs(int(robot_pos[1]*10))
    #         #deposit pheromones 
    #         self.pheromone_grids[robot_name][x][y] += self.intensity
    #         self.global_pheromone_grid[x][y] += self.intensity

    #         print(f"deposited in the grid: {x}, {y}" )    
    #         print(f"GRID (before sensing) for {robot_name}: ", self.pheromone_grids[robot_name])
    #         # print("GLOBAL PHEROMONE GRID", global_pheromone_grid)
    #     else:
    #         print("nothing deposited.")
#  GRID (before sensing) for e2:  [[0.0, 10.0, 173.0, 392.0, 197.0], [0.0, 64.0, 143.0, 317.0, 235.0], [14.0, 119.0, 397.0, 284.0, 124.0], [336.0, 479.0, 396.0, 87.0, 91.0], [448.0, 225.0, 19.0, 44.0, 31.0]]
# [[0.0, 0.0, 0.0, 108.0, 331.0], [0.0, 0.0, 35.0, 379.0, 144.0], [29.0, 124.0, 263.0, 228.0, 53.0], [285.0, 92.0, 203.0, 108.0, 0.0], [91.0, 195.0, 124.0, 0.0, 0.0]]
    
    def storingPosition(self, robot_name, robot_pos):
        # Store current and previous locations
        if robot_name not in self.robot_positions:  # Initialize storage for the robot
            self.robot_positions[robot_name] = {'current_pos': robot_pos, 'previous_pos': None}
            print(f"POSITION OF THE ROBOT {robot_name} : {self.robot_positions}")
        else:
            self.robot_positions[robot_name]['previous_pos'] = self.robot_positions[robot_name]['current_pos']  # Shift previous to old
            self.robot_positions[robot_name]['current_pos'] = robot_pos  # Update current
            print(f"POSITION OF THE ROBOT {robot_name} : {self.robot_positions}")
        
        return self.robot_positions

    def deposit_pheromone(self, robot_name, robot_pos):
        with open('pheromone_grid.txt', 'w') as file:
            #coordinate of deposition
            coordinate_key = self.getCoordinates()
            
            current_time = controller.time["current_time"]
            # previous_time = controller.time["previous_time"]
            # print("current time: ", current_time)
        
            if robot_name not in self.pheromone_grids:
                self.pheromone_grids[robot_name] = {}
            # print("pheromone grid: ", self.pheromone_grids)

            #storing robots position
            positions = self.storingPosition(robot_name, robot_pos)  

            distance_moved = self.has_moved(robot_name, positions[robot_name]['previous_pos'], positions[robot_name]['current_pos'])
            # print("has the robot moved to deposit:", distance_moved)
            if distance_moved:
                #deposit pheromones 
                # self.pheromone_grids[robot_name] = {self.intensity : [positions[robot_name]['current_pos'], current_time]}
                self.pheromone_grids[robot_name][current_time] = (self.intensity, positions[robot_name]['current_pos'])

                # print(f"deposited in the grid: {coordinate_key}" )    
                # print(f"GRID (before sensing) for {robot_name}: ", self.pheromone_grids[robot_name])
                print(f"GRID as of now: {current_time} ", self.pheromone_grids)
            else:
                print("nothing deposited.")
        
            # file.write("{}\n".format(self.pheromone_grids))
            json.dump(self.pheromone_grids, file)
        return self.pheromone_grids

    def calculateAngle(self, robot_pos, neighbour_pos):
        dx = robot_pos[0] - neighbour_pos[0]
        dy = robot_pos[1] - neighbour_pos[1]
        angle =  math.degrees(math.atan2(dy, dx))
        return angle
    
    def findNeighbours(self, robot_name, data, key, value):
        # robot_name = [name for name in data.keys()][0]
        neighbours = []
        #filter by distance 
        for timestamp, (intensity, coordinate) in data[robot_name].items():
            distance = math.sqrt(sum((a - b)**2 for a, b in zip(value[1], coordinate)))
            if distance <= 0.1 : #distance tolerance
                neighbours.append((timestamp, (intensity, coordinate)))
        #filter by time
        recent_neighbours = [n for n in neighbours if abs(n[0] - key) <= 0.2 ]
        return recent_neighbours
    
    # STEP-2
    def sense_pheromone(self, robot_name, robot_pos):

        neighbours = self.findNeighbours(robot_name,self.pheromone_grids, current_key, curent_value)
        neighbour_sensed_pheromone = {'current':[], 'front': [], 'back':[], 'left': [],'right':[]}
        # with open('pheromone_grid.txt', 'r') as file:
        #     # grid = file.read()
        #     grid = json.load(file)
        # # print("grid:", grid)
            
        #current pheromones by current position:
        for key, value in  self.pheromone_grids[robot_name].items():
            curent_value = value
            current_key = key
            if self.robot_positions[robot_name]['current_pos'] == value[1] and controller.time["current_time"] == key :
                self.current_sensed_pheromone = value[0]
        print("current_sensed_pheromone", self.current_sensed_pheromone, "of the key: ", current_key)
        neighbour_sensed_pheromone['current'].append((current_key, self.pheromone_grids[robot_name][current_key] ) )
        

        # for key, value in grid[robot_name].items():
        #     curent_value = value
        #     current_key = key
        #     print("value:", curent_value)
        #     print("key, ", current_key)
        #     if self.robot_positions[robot_name]['current_pos'] == value[1] and controller.time["current_time"] == key :
        #         self.current_sensed_pheromone = value[0]
        # print("current_sensed_pheromone", self.current_sensed_pheromone, "of the key: ", current_key)

        timestaps = sorted(self.pheromone_grids[robot_name].keys())
        current_index = timestaps.index(current_key)
        # print("current index:", current_index)
        previous_time = timestaps[current_index - 1] 
        neighbour_sensed_pheromone['back'].append((previous_time, self.pheromone_grids[robot_name][previous_time] ) )
        
        left_coordinate = [robot_pos[0], robot_pos[1]-1]
        right_coordinate = [robot_pos[0], robot_pos[1]+1]
        
        for timestap, (intensity, coord) in neighbours:
            dx = coord[0] - robot_pos[0] 
            dy = coord[1] - robot_pos[1] 

            if dy >= 0 : #TODO: check this condition
                # print('front')
                neighbour_sensed_pheromone['front'].append((timestap, (intensity,coord) ) )
            if (coord[0] == left_coordinate[0]) and (coord[1] == left_coordinate[1]) :#on left side
                # print('left')
                neighbour_sensed_pheromone['left'].append((timestap, (intensity,coord) ) )
            if (coord[0] == right_coordinate[0]) and (coord[1] == right_coordinate[1]) :#on left side
                # print('right')
                neighbour_sensed_pheromone['right'].append((timestap, (intensity,coord) ) )

        # sensed_pheromone = [current_sensed_pheromone] + neighbour_sensed_pheromone
        # print("neighbours without class", neighbours)
        print("neighbours with classification", neighbour_sensed_pheromone)
        # print("sensed_pheromone: ", sensed_pheromone)               
        return neighbour_sensed_pheromone
        

    # # STEP-2
    # def sense_pheromone(self, robot_name, robot_pos):
        
    #     #current pheromones by current position:
    #     current_pos = []
    #     for key, value in  self.pheromone_grids[robot_name].items():
    #         my_value = value
    #         my_key = key
    #         if self.robot_positions[robot_name]['current_pos'] == value[1] and controller.time["current_time"] == key :
    #             current_sensed_pheromone = value[0]
    #             current_pos = value[1]
    #     print("current_sensed_pheromone", current_sensed_pheromone)
    #     print("position of the current  pheromone: ", current_pos)
    #     print(my_value, my_key)

    #     self.findNeighbours(robot_name,self.pheromone_grids, my_key, my_value)
            

    #     """ THE DRUL COORDINATE SYSTEM
    #     It is a coordinate system designed for detecting neighbouring cells of the current position. It 
    #     detects up, down, left and right cells.  """

    #     #calculate the pheromone concentration at neighbouring cells (up, down left, right -> the list is in this order)
    #     neighbour_sensed_pheromone = []

    #     # absolue value of the coordinates of the robot for neighbour cells 
    #     x_n,y_n = abs(int(current_pos[0]*10)), abs(int(current_pos[1]*10))
    #     print("coordinate for neightbour detection : ", x_n, y_n)
    #     neighbors = [
    #         (x_n -1 , y_n),  # Up
    #         (x_n +1 , y_n),  # Down
    #         (x_n, y_n -1 ),  # Left
    #         (x_n, y_n + 1),  # Right
    #     ]

    #     # fixed the problem of not including -ve values and 10   
    #     #new problem : refer to the notebook

    #     # valid_neighbours = [(neighbor_x, neighbor_y) for neighbor_x, neighbor_y in neighbors if 0 <= neighbor_x < 10 and 0 <= neighbor_y < 10]
    #     valid_neighbours = [(neighbor_x, neighbor_y) for neighbor_x, neighbor_y in neighbors if 0 <= neighbor_x < 5 and 0 <= neighbor_y < 5]

    #     for neighbor_x, neighbor_y in valid_neighbours:
    #             # print("X : ", neighbor_x, "Y : ", neighbor_y)
    #             neighbour_sensed_pheromone.append(self.pheromone_grids[robot_name][neighbor_x][neighbor_y])

    #     # on hold that whether to increase the intensity where the robot is going after the detection
    #     # pheromone_grids[robot_name][x_c][y_c] += 1.0

    #     sensed_pheromone = [current_sensed_pheromone] + neighbour_sensed_pheromone

    #     print(f"GRID (after sensing) of {robot_name}: ", self.pheromone_grids[robot_name])        
        
    #     return sensed_pheromone

    
    
    # def adjust_robot_behavior(sensed_pheromone, robot_name):
    #     """
    #     Modifies the robot's movement based on the sensed pheromone values.

    #     Args:
    #         sensed_pheromone (list): A list containing the current and neighboring pheromone values.
    #         robot_name (str): The name of the robot.

    #     Returns:
    #         tuple: A tuple containing the new adjusted speed for the left and right motors of robot.
    #     """

    #     #index of the max pheromone
    #     max_pheromone_index = sensed_pheromone.index(max(sensed_pheromone))
    #     max_pheromone = max(sensed_pheromone)
        
    #     adjusted_speeds= [0.0,0.0]

    #     # print("MAX PHEROMONE DETECTED at index: ", max_pheromone)

    #     if max_pheromone > 0.5:  # Threshold to determine whether pheromones are present ; threshold = 0.5
    #         # Perform some behavior when pheromones are detected
    #         print(f"Pheromones detected for {robot_name} with highest being {max_pheromone}, adjusting behavior...")

    #         if max_pheromone_index in [0,1,2]: #pheromone is highest in its current position 
    #             print("this is highest, no change needed")
    #             if max_pheromone_index == 1:
    #                 adjusted_speeds = [MAX_SPEED * 0.7, MAX_SPEED] # slightly take right
    #             elif max_pheromone_index == 2:
    #                 adjusted_speeds = [MAX_SPEED, MAX_SPEED * 0.7] # slightly take left

    #         elif max_pheromone_index == 3: #pheromone is highest in left position 
    #             print("this is highest, in left direction")
    #             adjusted_speeds = [MAX_SPEED * 0.3, MAX_SPEED]

    #         elif max_pheromone_index == 4: #pheromone is highest in right position 
    #             print("this is highest, in right direction")
    #             adjusted_speeds = [MAX_SPEED, MAX_SPEED * 0.3]

    #     #adjusting obstacle avoidance 
    #     for j in range(8):
    #         if max_pheromone_index == 3:
    #             #modify left with more on right
    #             adjusted_speeds[0] = 1.0 - (distance_sensors_values[j]/ (1024/2)) 
    #             adjusted_speeds[1] = 1.2 - (distance_sensors_values[j]/ (1024/2)) 
    #         elif max_pheromone_index == 4:
    #             #modify right with more on left
    #             adjusted_speeds[0] = 1.2 - (distance_sensors_values[j]/ (1024/2)) 
    #             adjusted_speeds[1] = 1.0 - (distance_sensors_values[j]/ (1024/2)) 
    #         else:
    #             adjusted_speeds[0] = 1.0 - (distance_sensors_values[j]/ (1024/2)) 
    #             adjusted_speeds[1] = 1.0 - (distance_sensors_values[j]/ (1024/2)) 

    #     print("adjusted speed: ", adjusted_speeds)
    
    #     adjusted_left_speed = min(adjusted_speeds[0], MAX_SPEED)
    #     adjusted_right_speed = min(adjusted_speeds[1], MAX_SPEED)
                    
    #     # left_motor.setVelocity(adjusted_left_speed)
    #     # right_motor.setVelocity(adjusted_right_speed)
    #     return adjusted_left_speed, adjusted_right_speed

    def get_food_position(self):
        food_node = robot.supervisor.getFromDef('food')
        position = food_node.getField('translation').getSFVec3f()
        return position

    def adjust_robot_behavior(self, sensed_pheromone, robot_name):
        print("ADJSUTING ROBOT BEHAVIOUR>>>>>>>>>")

        # pid_controller  = PIDController(0.1,0.01,0.005)
        current = self.robot_positions[robot_name]['current_pos']
        target = self.get_food_position()
        pid_controller  = PIDController(current, target, 1.0,0.01,0.01, np.radians(90))
        v, turn_adjustment = pid_controller.update()
    
        max_intensity = None
        # max_intensity_data = None
        max_direction = None
        for key, value_list in sensed_pheromone.items():
            for item in value_list:
                # print("intensity,", item)
                current_val = item[1][0]
                if max_intensity is None or current_val > max_intensity:
                    max_intensity = current_val
                    max_intensity_data = item[1]
                    max_direction = key

        # print(f"max pheromone out of the neighbours; {max_intensity}, associated data: {max_intensity_data}")
        # print(f"max direction: {max_direction}")

        if max_direction == 'left':
            print("this is highest, in left direction")
            adjusted_left_speed = max(0, robot.MAX_SPEED - abs(turn_adjustment))
            adjusted_right_speed = robot.MAX_SPEED
        elif max_direction == 'right':
            print("this is highest, in right direction")
            adjusted_right_speed = max(0,robot.MAX_SPEED - abs(turn_adjustment))
            adjusted_left_speed = robot.MAX_SPEED
        else:
            print("this is highest, no change needed")
            adjusted_right_speed = v
            adjusted_left_speed = v
        
        for j in range(8):
            adjusted_left_speed += robot.coefficients[j][0] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))
            adjusted_right_speed += robot.coefficients[j][1] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))

        # print("adjusted speed: ", adjusted_left_speed, adjusted_right_speed)
        return adjusted_left_speed, adjusted_right_speed
   
    #STEP-4
    def evaporate_pheromone(self, robot_name, elapsed_time, intensity, evaporation_rate=0.1):
        #decay factor 
        decay_factor = math.exp(-evaporation_rate * elapsed_time)
        decayed_intensity = 0.0
        original_intensity = intensity
        decayed_intensity = original_intensity * decay_factor
        return decayed_intensity    


class Controller():
    def __init__(self, robot, pheromone_algorithm=None) -> None:
        self.pheromone_algorithm = pheromone_algorithm
        self.robot = robot

        self.present_time = 0
        self.senses_food = False
        self.robot_state = RobotState.SEARCHING

        self.last_sensing_time = 0
        self.last_evp_time = 0
        self.sensing_started = False
        self.robot_pos = 0

        self.displayA = robot.supervisor.getDevice("displayA")  
        self.displayB = robot.supervisor.getDevice("displayB")  
        self.x_trajectory = []
        self.y_trajectory = []
        self.intensities = {}

        self.time = {'current_time': robot.supervisor.getTime(), 'previous_time': None} 
    
        #start postion 
        self.start_position = self.pheromone_algorithm.get_robot_position()
        self.current_position = 0
        self.goal = 0

        self.elapsed_time_evaporation = 0


    def startPheromone(self, robot_names):
        # global last_sensing_time, last_evp_time, robot_pos
        #for two robot - sensing after 5-10 secs of start and continue that
        
        for robot_name in robot_names:
            if robot.supervisor.getName() == robot_name:
                # print("here the robot posotion is geting recorded...")
                self.robot_pos = self.pheromone_algorithm.get_robot_position()
                self.current_position = self.robot_pos

                #DEPOSITING
                print("DEPOSITING")
                self.pheromone_algorithm.deposit_pheromone(robot_name, self.robot_pos)

                #SENSING
                # elapsed_time_last_sensing = time["current_time"] - last_sensing_time
                if self.present_time >= 5:
                    print("sensing...")
                    sensing = self.pheromone_algorithm.sense_pheromone(robot_name, self.robot_pos)
                    new_left_speed, new_right_speed = self.pheromone_algorithm.adjust_robot_behavior(sensing, robot_name) 
                    # self.pheromone_algorithm.adjust_robot_behavior(sensing, robot_name)                     
                    robot.speeds[0] = new_left_speed
                    robot.speeds[1] = new_right_speed
                    robot.left_motor.setVelocity(robot.speeds[0])
                    robot.right_motor.setVelocity(robot.speeds[1])
                    # print("Left side speed: ", left_motor.getVelocity(), "right side speed: ", right_motor.getVelocity())
                    self.last_sensing_time = self.time["current_time"]

        # #EVAPORATING               
        key_to_delete = []
        for timestamp, (intensity, coord) in self.pheromone_algorithm.pheromone_grids[robot_name].items():
            deposition_time  = timestamp
            elapsed_time = self.time["current_time"] - deposition_time
            # value = (intensity, coord)
            if 2.0 <= elapsed_time <= 4.0:
                # print("evaporating...")   
                evaporated_pheromone = self.pheromone_algorithm.evaporate_pheromone(robot_name, elapsed_time, intensity)
                self.pheromone_algorithm.pheromone_grids[robot_name][timestamp] = (evaporated_pheromone, coord)
                if evaporated_pheromone <= 0.1:
                    key_to_delete.append(timestamp)

        for time in key_to_delete:
            del self.pheromone_algorithm.pheromone_grids[robot_name][time]        

    def grabbing(self):
        if robot.get_camera_input():
            if 65000 < number_pixels < 517140: #66080
                print("seeing food")
                self.senses_food = True
            else:
                self.senses_food = False
        else:
            self.senses_food = False

        print("FOOD GRABBED", self.senses_food)
        return self.senses_food
    
    def distanceToHome(self, current_position):
        distance_to_start = math.sqrt((self.start_position[0] - current_position[0])**2 + 
                                  (self.start_position[1] - current_position[1])**2)
        return distance_to_start

    def homing(self):
        self.goal = self.pheromone_algorithm.get_food_position()
        print("Start position: ", self.start_position)
        print("current position: ", self.current_position)

        homing_vector = [self.start_position[i] - self.current_position[i] for i in range(2)]
        # print(" homing vector: ", homing_vector)
        
        distance_to_start = self.distanceToHome(self.current_position)
        print("distance to start :", round(distance_to_start,2))

        #robots direction - direction of the vector from the current position to the target relative to the coordinate system,
        angle_to_start = math.atan2(homing_vector[1], homing_vector[0])
        # print("angle to start(type dont know): ", round(angle_to_start,2))

        if 0.05 <= round(distance_to_start,2) <= 0.06:  #values kept just by testing, adjust this threshold based on accuracy
            print("Reached home. Stopping!")
            robot.left_motor.setVelocity(0.0)
            robot.right_motor.setVelocity(0.0)
            sys.exit(0) 
        else:
            print("Not yet at home.")
            if round(angle_to_start,2) > 0.1:
                robot.turn_left()
            else:         
                robot.turn_right()
            robot.go_forward()

    def reachedHome(self):
        if self.current_position == self.start_position: 
                # print("Reached home. Stopping!")
                # robot.left_motor.setVelocity(0.0)
                # robot.right_motor.setVelocity(0.0)
                # sys.exit(0)
            return True 
        else:
            return False
        
    def updateRobotState(self):
        if self.robot_state == RobotState.SEARCHING:
            print("12345")
            #in searching mode, grab food
            if self.grabbing():
                #once food is found chnage the state 
                self.robot_state = RobotState.HOMING
        elif self.robot_state == RobotState.HOMING:
            print("00000")
            self.reachedHome()
            # self.robot_state = RobotState.IDLE
    
    #---------DISPLAY THE ROBOT'S TRAJECTORY----------------------------------------
    def mappingTrajectory(self, robot_names):   
        #plotting graph 
        current_position = self.pheromone_algorithm.get_robot_position()
        self.x_trajectory.append( current_position[0] )
        self.y_trajectory.append( current_position[1] )

        plt.plot(self.x_trajectory,self.y_trajectory)
        plt.title("ROBOT TRAJECTORY")
        plt.xlabel("X_AXIS")
        plt.ylabel("Y_AXIS")
        plt.grid(True)
        plt.savefig("trajectory.png")

        image = self.displayA.imageLoad( "trajectory.png" )
        self.displayA.imagePaste(image, 0,0, False)
        self.displayA.imageDelete(image)
    
    def mappingConcentration(self, robot_names):   
        # width = self.display.getWidth()
        # height = self.display.getHeight()
        #plotting graph 
        current_position = self.pheromone_algorithm.get_robot_position()
        self.x_trajectory.append( current_position[0] )
        self.y_trajectory.append( current_position[1] )

        # print("GRIDDDDDDDDDDDDDDDD, ", self.pheromone_algorithm.pheromone_grids)
        for robot_name,timestamp_data in self.pheromone_algorithm.pheromone_grids.items():
            self.intensities[robot_name] = []
            for time, (intensity, coord) in timestamp_data.items():
                # print("timestamp, ", time, "int", intensity, "coord", coord)
                self.intensities[robot_name].append(intensity)
        # print("GOT INTTTT", self.intensities)
        for robot_name in robot_names:
            if robot_name == robot.supervisor.getName():
                plt.plot(self.x_trajectory,self.y_trajectory, label="ROBOT TRAJECTORY")
                plt.plot(self.intensities[robot_name], label="Pheromone Intensity" )
                plt.legend()
                plt.title(f"ROBOT TRAJECTORY and Pheromone Intensity of {robot_name}")
                plt.xlabel("X_AXIS")
                plt.ylabel("Y_AXIS")
                plt.grid(True)
                plt.savefig("trajectory&pheromone.png")

                image = self.displayB.imageLoad( "trajectory&pheromone.png" )
                self.displayB.imagePaste(image, 0,0, False)
                self.displayB.imageDelete(image)
        

      
    def main(self):  
        while robot.supervisor.step(robot.timestep) != -1:
            robot.reset_actuator_values()
            robot.get_sensor_input()
            robot.blink_leds()
            robot_names = ['e1', 'e2']

            self.present_time = robot.supervisor.getTime()
            self.mappingTrajectory(robot_names)
            

            #to record  the time when each robot starts moving 
            self.time['previous_time'] = self.time['current_time']  # Shift previous to old
            self.time['current_time'] = self.present_time # Update current
            # print("TIME: ", time)

            #random movement of the robot (kind of avoids obstacles)
            for i in range(2):
                robot.speeds[i] = 0.0
                for j in range(8):
                    robot.speeds[i] += robot.coefficients[j][i] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))

            # print("main loop robot state: ", self.robot_state)
            self.updateRobotState()
            
            
            if self.robot_state == RobotState.SEARCHING:  
                if self.present_time >= 5:
                    print("started pheromone")
                    #pheromone algorithm works after 5 secs of scattering
                    self.startPheromone(robot_names)
                    self.mappingConcentration(robot_names)
                else:
                    #random walk for less than 5 secs 
                    # if wall_detected() or cliff_detected():
                    # print("wall in front")
                    print("started random")
                    left_speed = robot.speeds[0]
                    right_speed = robot.speeds[1]
                    robot.left_motor.setVelocity(left_speed)
                    robot.right_motor.setVelocity(right_speed)
                self.grabbing()
            
            elif self.robot_state == RobotState.HOMING:
                print("going home")   
                self.homing()

            robot.set_actuators()
            robot.step()


if __name__ == "__main__":
    robot = InitializeRobot()    
    pheromone = PheromoneAlgorithm(robot)
    controller = Controller(robot, pheromone)
    pheromone.setController(controller )
    controller.main()





    # [(5.856, (1.0, [0.19125643094467945, -0.4109450433113728, -6.396149301078104e-05])), 
    # (5.92, (1.0, [0.1895099596445636, -0.410935954134034, -6.395547291541091e-05])), 
    # (5.984, (1.0, [0.1877603893884551, -0.41092668499391577, -6.394562073725775e-05])), 
    # (6.048, (1.0, [0.18601009333544283, -0.41091707848145886, -6.394137996434263e-05]))]