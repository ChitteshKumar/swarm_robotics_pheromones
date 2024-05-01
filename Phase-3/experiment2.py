"""pheromone_algo_supervisor.py controller."""
import numpy as np
from controller import Motor, Field,DistanceSensor, LED, Node, Camera, Emitter, Receiver, Supervisor
import sys
import cv2  # OpenCV for image processing
import math
import json
import matplotlib.pyplot as plt 
from scipy.ndimage import gaussian_filter

class InitializeRobot():
    def  __init__(self, name=None, supervisor=None):
        # create the Robot instance.
        # robot = Robot()
        self.supervisor = supervisor
        self.name = name
        #time step of the world
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.device_map = {
            'e1': {
                'distance_sensor_names': ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"], 
                'ground_sensor_names': ["gs0", "gs1", "gs2"],
                'led_names' : ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"],
                "camera" : "camera",
                "emitter1" : "emitter",
                "receiver1" : "receiver",
                "left_motor" : "left wheel motor",
                "right_motor" : "right wheel motor"
            },
            'e2': {
                'distance_sensor_names': ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"], 
                'ground_sensor_names': ["gs0", "gs1", "gs2"],
                'led_names' : ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"],
                "camera" : "camera",
                "emitter2" : "emitter",
                "receiver2" : "receiver",
                "left_motor" : "left wheel motor",
                "right_motor" : "right wheel motor"
            }
        } 

        self.initializeDevices()
    
    def initializeDevices(self):
            #Distance Sensor in the robot
            self.distance_sensors =  [None] * 8
            self.distance_sensors_values = [0.0] * 8  # Initialize distance_sensors_values 

            for i, name in enumerate(self.device_map[self.name]['distance_sensor_names']):
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
                        # for i, name in enumerate(self.device_map[self.name]['ground_sensor_names']):
                        #     self.ground_sensors[i] = self.supervisor.getDevice(name)
                        #     self.ground_sensors[i].enable(self.timestep)
                        self.ground_sensors[id] = self.supervisor.getDevice(ground_sensor_names[id])
                        self.ground_sensors[id].enable(self.timestep)

            # print(f"GORUND SESNROS {self.name}", self.ground_sensors)
            
            #Led in the device
            self.leds = [None] * 10
            self.leds_values = [False] * 10  # Initialize leds_values
            for i, name in enumerate(self.device_map[self.name]['led_names']):
                self.leds[i] = self.supervisor.getDevice(name)
            
            #sensor for camera
            # self.camera_e1 = self.supervisor.getDevice(self.device_map[self.name]["camera"])
            # self.camera_e1.enable(self.timestep)

            # self.camera_e2 = self.supervisor.getDevice(self.device_map[self.name]["camera"])
            # self.camera_e2.enable(self.timestep)

            self.camera = self.supervisor.getDevice(self.device_map[self.name]["camera"])
            self.camera.enable(self.timestep)

    
            # Define emitter and receiver
            # self.emitter = self.supervisor.getDevice(self.device_map[self.name]["emitter"])
            # receiver = self.supervisor.getDevice(self.device_map[self.name]["receiver"])
            # receiver.enable(self.timestep)
            
            #Initialising the motors 
            self.left_motor = self.supervisor.getDevice(self.device_map[self.name]["left_motor"])
            self.right_motor = self.supervisor.getDevice(self.device_map[self.name]["right_motor"])
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
        for i in range(10):
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
        self.leds_values[(self.counter // 10) % 10] = True
    
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

    # def get_camera_input(self):
    #     global number_pixels, number_brown_pixels
    #     # Define the range of red color in HSV
    #     lower_red = np.array([0, 150, 150])
    #     upper_red = np.array([10, 255, 255])

    #     lower_brown = np.array([10, 100, 20])
    #     upper_brown = np.array([30, 255, 200])

    #     # For e1 robot
    #     image_e1 = self.camera_e1.getImage()
    #     width_e1, height_e1 = self.camera_e1.getWidth(), self.camera_e1.getHeight()
    #     frame_e1 = np.frombuffer(image_e1, dtype=np.uint8).reshape((height_e1, width_e1, 4))

    #     hsv = cv2.cvtColor(frame_e1, cv2.COLOR_BGR2HSV)
    #     mask = cv2.inRange(hsv, lower_red, upper_red)
    #     is_red = np.any(mask)
    #     number_pixels = np.sum(mask)
    #     print("Number of red pixels by robot e1:", number_pixels)

    #     #For e2 robot
    #     image_e2 = self.camera_e2.getImage()
    #     width_e2, height_e2 = self.camera_e2.getWidth(), self.camera_e2.getHeight()
    #     frame_e2 = np.frombuffer(image_e2, dtype=np.uint8).reshape((height_e2, width_e2, 4))

    #     hsv = cv2.cvtColor(frame_e2, cv2.COLOR_BGR2HSV)
    #     mask = cv2.inRange(hsv, lower_red, upper_red)
    #     is_red = np.any(mask)
    #     number_pixels = np.sum(mask)
    #     print("Number of red pixels by robot e2:", number_pixels)

    #     #detecting brown
    #     brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)
    #     is_brown = np.any(brown_mask)
    #     number_brown_pixels = np.sum(brown_mask)
    #     print("Number of brown pixels:", number_brown_pixels)

    #     return is_red, is_brown

    def get_camera_input(self):
        global number_pixels, number_brown_pixels
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
        print("mask", mask)
        # Check if there are any red pixels in the image
        is_red = np.any(mask)
        number_pixels = np.sum(mask)
        print(f"Number of red pixels by {self.name}:", number_pixels)

        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([30, 255, 200])
        brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)
        #detecting brown
        is_brown = np.any(brown_mask)
        number_brown_pixels = np.sum(brown_mask)
        print(f"Number of brown pixels by {self.name}:", number_brown_pixels)
        print("issss reddddddddddd:", is_red)
        return is_red, is_brown
    
    
    def cliff_detected(self):
         for i in range(3):
             if self.ground_sensors[i] and self.ground_sensors[i] < 100.0:
                 return True
         return False
    
    def wall_detected(self):
        # Check if any distance sensor reading indicates a wall
        for i in range(8):
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
        # output =  self.kp * ep+ self.ki * ei + self.kd * ed
        output =  self.kp * ep + self.kd * ed
        output = math.atan2(np.sin(output), np.cos(output))
        # print("angle to start", output, angle_to_start)

        self.E = self.E + e
        self.old_e = e
        v = self.desiredV

        return v, output       

global_pheromone_grid = {}
class PheromoneAlgorithm():

    def __init__(self, robot) -> None:
        #define pheromone properties
        self.exploration_intensity = 1.0
        self.higherIntensity = 20.0
        self.homing_intensity = 5.0
        self.distances = {}

        self.pheromone_grids = {}
        
        self.robot_positions = {}

        self.evaporation_rate = 0.1
        self.evaporation_interval = 2.0 # in seconds

        self.robot = robot
        self.controller = None

        self.current_sensed_pheromone = 0
        self.start_position = self.get_robot_position()
        self.adjusted_left_speed = 0
        self.adjusted_right_speed = 0

        #food collected
        # self.global_food_collected = 0
        self.global_food_collected = {'e1': 0, 'e2':0}
        
    def setController(self, controller):
        self.controller = controller

    def get_robot_position(self):
        robot_node = self.robot.supervisor.getSelf()
        # robot_node = self.robot.supervisor.getFromDef()
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
        print(f"distance from the previous pos {robot_name}: ", distance)
        return distance >= threshold
    
    def getCoordinates(self):
        coordinate = self.get_robot_position()
        return coordinate
    
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
        # if self.controller.robot_state == RobotState.SEARCHING:
        if self.controller.robot_states[self.controller.robot_name] == RobotState.SEARCHING:
            with open('global_pheromone_grid.txt', 'w') as file:
                #coordinate of deposition
                coordinate_key = self.getCoordinates()
                
                current_time = controller.time["current_time"]
                # previous_time = controller.time["previous_time"]
                # print("current time: ", current_time)
            
                if robot_name not in self.pheromone_grids:
                    self.pheromone_grids[robot_name] = {}

                if robot_name not in global_pheromone_grid:
                    global_pheromone_grid[robot_name] = {}

                #storing robots position
                positions = self.storingPosition(robot_name, robot_pos)  

                distance_moved = self.has_moved(robot_name, positions[robot_name]['previous_pos'], positions[robot_name]['current_pos'])
                # print("has the robot moved to deposit:", distance_moved)
                
                if distance_moved:
                    
                    #deposit pheromones 
                    global_pheromone_grid[robot_name][current_time] = (self.exploration_intensity, positions[robot_name]['current_pos'])
                    # print("global pheromone grid: ", global_pheromone_grid)

                    self.pheromone_grids[robot_name][current_time] = (self.exploration_intensity, positions[robot_name]['current_pos'])

                    if (len(global_pheromone_grid[robot_name]) )%3 == 0:
                        # print("len", len(self.global_pheromone_grid[robot_name]))
                        self.pheromone_grids[robot_name][current_time] = (self.higherIntensity, positions[robot_name]['current_pos'])

                    # print(f"GRID as of now: {current_time} of {robot_name}", self.pheromone_grids)
                else:
                    print("nothing deposited.")
            
                file.write("{}\n".format(global_pheromone_grid))
                # json.dump(self.global_pheromone_grid, file)
            
            return self.pheromone_grids
        
        elif self.controller.robot_states[self.controller.robot_name] == RobotState.HOMING:
            
            current_time = controller.time["current_time"]
            # previous_time = controller.time["previous_time"]
            # print("current time: ", current_time)
        
            if robot_name not in self.pheromone_grids:
                self.pheromone_grids[robot_name] = {}

            if robot_name not in global_pheromone_grid:
                    global_pheromone_grid[robot_name] = {}

            #storing robots position
            home_positions = self.storingPosition(robot_name, robot_pos)  

            distance_moved = self.has_moved(robot_name, home_positions[robot_name]['previous_pos'], home_positions[robot_name]['current_pos'])
            print(f"has the robot {robot_name} moved to deposit:", distance_moved)
            if distance_moved:
                #deposit pheromones 
                global_pheromone_grid[robot_name][current_time] = (self.homing_intensity, home_positions[robot_name]['current_pos'])

                self.pheromone_grids[robot_name][current_time] = (self.homing_intensity, home_positions[robot_name]['current_pos'])

                # if (len(self.global_pheromone_grid[robot_name]) )%3 ==0:
                #     print("len", len(self.global_pheromone_grid[robot_name]))
                #     self.pheromone_grids[robot_name][current_time] = (self.higherIntensity, positions[robot_name]['current_pos'])

                print(f"HOME GRID as of now: {current_time} for robot {robot_name} ", self.pheromone_grids)
            else:
                print("nothing deposited.")
            
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
         
        #current pheromones by current position:
        for key, value in  self.pheromone_grids[robot_name].items():
            self.curent_value = value
            self.current_key = key
            if self.robot_positions[robot_name]['current_pos'] == value[1] and controller.time["current_time"] == key :
                self.current_sensed_pheromone = value[0]
        print("current_sensed_pheromone", self.current_sensed_pheromone, "of the key: ", self.current_key, f"by the robot {robot_name}")

        neighbours = self.findNeighbours(robot_name,self.pheromone_grids, self.current_key, self.curent_value)
        neighbour_sensed_pheromone = {'current':[], 'front': [], 'back':[], 'left': [],'right':[]}
       
        neighbour_sensed_pheromone['current'].append([self.current_key, self.pheromone_grids[robot_name][self.current_key] ])
        
        timestaps = sorted(self.pheromone_grids[robot_name].keys())
        current_index = timestaps.index(self.current_key)
        previous_time = timestaps[current_index - 1] 
        neighbour_sensed_pheromone['back'].append([previous_time, self.pheromone_grids[robot_name][previous_time] ] )
        
        left_coordinate = [robot_pos[0], robot_pos[1]-1]
        right_coordinate = [robot_pos[0], robot_pos[1]+1]
        
        for timestap, (intensity, coord) in neighbours:
            dx = coord[0] - robot_pos[0] 
            dy = coord[1] - robot_pos[1] 

            if dy >= 0 : #TODO: check this condition
                # print('front')
                neighbour_sensed_pheromone['front'].append([timestap, (intensity,coord) ])
            if (coord[0] == left_coordinate[0]) and (coord[1] == left_coordinate[1]) :#on left side
                # print('left')
                neighbour_sensed_pheromone['left'].append( [timestap, (intensity,coord) ] )
            if (coord[0] == right_coordinate[0]) and (coord[1] == right_coordinate[1]) :#on left side
                # print('right')
                neighbour_sensed_pheromone['right'].append([timestap, (intensity,coord) ] )

        print("neighbours with classification", neighbour_sensed_pheromone)         
        return neighbour_sensed_pheromone

    def get_food_position(self):
        food_node = robot.supervisor.getFromDef('food')
        position = food_node.getField('translation').getSFVec3f()
        return position
    
    def distanceToHome(self, start_pos, current_pos):
        distance_to_start = math.sqrt((start_pos[0] - current_pos[0])**2 + 
                                  (start_pos[1] - current_pos[1])**2)
        return distance_to_start
    
    def adjust_robot_behavior(self, sensed_pheromone, robot_name, robot_state):
        print("State right now in adjusting", robot_state )

        # if self.controller.robot_states[self.controller.robot_name] == RobotState.SEARCHING:
        if robot_state == RobotState.SEARCHING:
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
                self.adjusted_left_speed = max(0, robot.MAX_SPEED - abs(turn_adjustment))
                self.adjusted_right_speed = robot.MAX_SPEED
            elif max_direction == 'right':
                print("this is highest, in right direction")
                self.adjusted_right_speed = max(0,robot.MAX_SPEED - abs(turn_adjustment))
                self.adjusted_left_speed = robot.MAX_SPEED
            else:
                print("this is highest, no change needed")
                self.adjusted_right_speed = v
                self.adjusted_left_speed = v
            
            for j in range(8):
                self.adjusted_left_speed += robot.coefficients[j][0] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))
                self.adjusted_right_speed += robot.coefficients[j][1] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))

            # print("adjusted speed: ", adjusted_left_speed, adjusted_right_speed)
            # return self.adjusted_left_speed, self.adjusted_right_speed
        
        elif robot_state == RobotState.HOMING:
      
            current = self.robot_positions[robot_name]['current_pos']
            target = self.start_position
            # distance_to_start = self.distanceToHome(self.start_position, self.current_position)
            distance_to_start = self.distanceToHome(current, target)
            print("distance to start :", round(distance_to_start,2))

            if not self.controller.reachedHome(distance_to_start):

                pid_controller  = PIDController(current, target, 1.0,0.01,0.01, np.radians(90))
                v, turn_adjustment = pid_controller.update()
                # print("angle to start: ", round(turn_adjustment,2))

                if distance_to_start <= 0.7:
                    print("Not yet at home.")
                    if -2.0 < round(turn_adjustment,2) < 0:
                        print("left direction")
                        self.adjusted_left_speed = 0.0
                        self.adjusted_right_speed = robot.MAX_SPEED

                    elif round(turn_adjustment,2) < -2.0:         
                        print("right direction")
                        # self.adjusted_right_speed = max(0,(robot.MAX_SPEED) - abs(turn_adjustment))
                        self.adjusted_right_speed = 0.0
                        self.adjusted_left_speed = robot.MAX_SPEED
                else:
                    print("this is highest, no change needed")
                    self.adjusted_right_speed = v
                    self.adjusted_left_speed = v
                
                for j in range(8):
                    self.adjusted_left_speed += robot.coefficients[j][0] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))
                    self.adjusted_right_speed += robot.coefficients[j][1] * (1.0 - (robot.distance_sensors_values[j]/ (1024/2)))
        
            else:
                print("Reached home. Stopping!")
                self.adjusted_left_speed = 0.0
                self.adjusted_right_speed = 0.0

                #resetting the food counter 
                self.global_food_collected[robot_name] += self.controller.food_counter
                # self.global_food_collected += self.controller.food_counter
                self.controller.food_counter = 0

                print("global food collected", self.global_food_collected, self.controller.food_counter)
                # sys.exit(0)

        return self.adjusted_left_speed, self.adjusted_right_speed
   
    #STEP-4
    def evaporate_pheromone(self, robot_name, elapsed_time, intensity, evaporation_rate=0.1):
        #decay factor 
        decay_factor = math.exp(-evaporation_rate * elapsed_time)
        decayed_intensity = 0.0
        original_intensity = intensity
        decayed_intensity = original_intensity * decay_factor
        return decayed_intensity    

# class Experiment():
#     def __init__(self, robot, pheromone_algorithm=None) -> None:
#         self.robot = robot
#         self.pheromone_algorithm = pheromone_algorithm
#         self.controller = None

#     def setController(self, controller):
#         self.controller = controller

#     def food_collected(self):
#         food_grabbed = self.controller.grabbing()
#         food_counter = 0
#         if food_grabbed:
#             food_counter += 1 

#         print("Food collected", food_counter)

#     def time_taken():
#         pass
#     def energy_used():
#         pass


class Controller():
    def __init__(self, robot, pheromone_algorithm=None, name=None):
        self.robot_name = name
        self.pheromone_algorithm = pheromone_algorithm
        self.robot = robot

        self.present_time = 0
        self.senses_food = False
        # self.robot_state = RobotState.SEARCHING
        
        self.robot_states = {}
        self.initializeState()

        self.last_sensing_time = 0
        self.last_evp_time = 0
        self.sensing_started = False
        self.robot_pos = 0

        self.displayA = robot.supervisor.getDevice("displayA")  
        self.displayB = robot.supervisor.getDevice("displayB")  
        self.x_trajectory = []
        self.y_trajectory = []
        self.intensities = {}
        self.current_positions_list = []

        self.time = {'current_time': robot.supervisor.getTime(), 'previous_time': None} 
    
        #start postion 
        self.start_position = self.pheromone_algorithm.get_robot_position()
        self.current_position = 0
        self.goal = 0
        self.isHoming = False

        self.elapsed_time_evaporation = 0

        #homing
        self.max_intensity = 0
        self.sensed_target_direction = None
        self.max_intensity_key = ""

        #food collected 
        self.food_grabbed = False
        # self.food_counter = 0
        self.food_counter = {'e1':0, 'e2':0}
        self.is_red, self.is_brown = None, None

    def startPheromone(self):
        # global last_sensing_time, last_evp_time, robot_pos
        #for two robot - sensing after 5-10 secs of start and continue that
        
        if robot.supervisor.getName() == self.robot_name :
            # print("here the robot posotion is geting recorded...")
            self.robot_pos = self.pheromone_algorithm.get_robot_position()
            self.current_position = self.robot_pos

            #DEPOSITING
            print("DEPOSITING")
            print("robot name coming here", self.robot_name)
            self.pheromone_algorithm.deposit_pheromone(self.robot_name, self.robot_pos)

            #SENSING
            # elapsed_time_last_sensing = time["current_time"] - last_sensing_time
            if self.present_time >= 5:
                print("sensing...")
                sensing = self.pheromone_algorithm.sense_pheromone(self.robot_name, self.robot_pos)
                new_left_speed, new_right_speed = self.pheromone_algorithm.adjust_robot_behavior(sensing, self.robot_name, self.robot_states[self.robot_name]) 
                # self.pheromone_algorithm.adjust_robot_behavior(sensing, robot_name)                     
                robot.speeds[0] = new_left_speed
                robot.speeds[1] = new_right_speed
                robot.left_motor.setVelocity(robot.speeds[0])
                robot.right_motor.setVelocity(robot.speeds[1])
                # print("Left side speed: ", left_motor.getVelocity(), "right side speed: ", right_motor.getVelocity())
                self.last_sensing_time = self.time["current_time"]

        # #EVAPORATING               
        key_to_delete = []
        for timestamp, (intensity, coord) in self.pheromone_algorithm.pheromone_grids[self.robot_name].items():
            deposition_time  = timestamp
            elapsed_time = self.time["current_time"] - deposition_time
            # value = (intensity, coord)
            if 2.0 <= elapsed_time <= 4.0:
                # print("evaporating...")   
                evaporated_pheromone = self.pheromone_algorithm.evaporate_pheromone(self.robot_name, elapsed_time, intensity)
                self.pheromone_algorithm.pheromone_grids[self.robot_name][timestamp] = (evaporated_pheromone, coord)
                if evaporated_pheromone <= 0.1:
                    key_to_delete.append(timestamp)

        for time in key_to_delete:
            del self.pheromone_algorithm.pheromone_grids[self.robot_name][time]        

    def grabbing(self):
        # self.is_red , _ = robot.get_camera_input()
        print("is redddddddddddd", self.is_red)
        if self.is_red:
            if 65000 < number_pixels < 517140: #66080
                print(f"seeing food by {self.robot_name}")
                self.senses_food = True
                
                #food counter
                self.food_counter[self.robot_name] += 1 
                print(f"Food collected by {self.robot_name}", self.food_counter)
                self.food_grabbed = True
            else:
                self.senses_food = False
        else:
            self.senses_food = False

        print("FOOD GRABBED", self.senses_food)
        return self.senses_food
    
    def distanceToHome(self, start_pos, current_pos):
        distance_to_start = math.sqrt((start_pos[0] - current_pos[0])**2 + 
                                  (start_pos[1] - current_pos[1])**2)
        return distance_to_start

    def homing(self):
        if self.robot_name == robot.supervisor.getName() and self.robot_states[self.robot_name]== RobotState.HOMING:

            self.robot_pos = self.pheromone_algorithm.get_robot_position()
            #deposit while homing... with different value 
            print("deposit while homing...")
            home_grid = self.pheromone_algorithm.deposit_pheromone(self.robot_name, self.robot_pos)

            #sense the highest neighbour 
            print("sensing home pheromones...")
            home_sensing = self.pheromone_algorithm.sense_pheromone(self.robot_name, self.robot_pos)

            print("adjusting behaviour for homing")
            new_left_speed, new_right_speed = self.pheromone_algorithm.adjust_robot_behavior(home_sensing, self.robot_name, self.robot_states[self.robot_name]) 
            robot.speeds[0] = new_left_speed
            robot.speeds[1] = new_right_speed
            robot.left_motor.setVelocity(robot.speeds[0])
            robot.right_motor.setVelocity(robot.speeds[1])
            # print("Left side speed: ", left_motor.getVelocity(), "right side speed: ", right_motor.getVelocity())
            self.last_sensing_time = self.time["current_time"]

        #EVAPORATING               
        key_to_delete = []
        for timestamp, (intensity, coord) in home_grid[self.robot_name].items():
            deposition_time  = timestamp
            elapsed_time = self.time["current_time"] - deposition_time
            # value = (intensity, coord)
            if 2.0 <= elapsed_time <= 4.0:
                # print("evaporating...")   
                evaporated_pheromone = self.pheromone_algorithm.evaporate_pheromone(self.robot_name, elapsed_time, intensity)
                home_grid[self.robot_name][timestamp] = (evaporated_pheromone, coord)
                if evaporated_pheromone <= 0.1:
                    key_to_delete.append(timestamp)

        for time in key_to_delete:
            del home_grid[self.robot_name][time]  
                
    def reachedHome(self,distance_to_start ):
        if self.current_position == self.start_position or 0.05 <= round(distance_to_start,2) <= 0.06 : 
            return True 
        else:
            return False
    
    def initializeState(self):
        if self.robot_name not in self.robot_states:
            self.robot_states[self.robot_name] = RobotState.SEARCHING
        print("initliazed states:", self.robot_states)

    # def updateRobotState(self):
    #     if self.robot_state == RobotState.SEARCHING:
    #         print("12345")
    #         #in searching mode, grab food
    #         # if self.grabbing():
    #         if self.senses_food:
    #             print("00000")
    #             #once food is found chnage the state 
    #             self.robot_state = RobotState.HOMING

    def updateRobotState(self):
        if self.robot_states[self.robot_name] == RobotState.SEARCHING:
            print("-----12345------")
            #in searching mode, grab food
            if self.grabbing():
            # if self.senses_food:
                print("---------00000----------")
                #once food is found chnage the state 
                self.robot_states[self.robot_name] = RobotState.HOMING
        
    
    
    def mappingTrajectory(self):   
        #plotting graph 
        current_position = self.pheromone_algorithm.get_robot_position()
        self.x_trajectory.append( current_position[0] )
        self.y_trajectory.append( current_position[1] )

        plt.plot(self.x_trajectory,self.y_trajectory)
        plt.title("ROBOT TRAJECTORY")
        plt.xlabel("X_AXIS")
        plt.ylabel("Y_AXIS")
        plt.grid(True)
        plt.savefig("trajectory_pheromone.png")

        image = self.displayA.imageLoad( "trajectory_pheromone.png" )
        self.displayA.imagePaste(image, 0,0, False)
        self.displayA.imageDelete(image)
    
    def mappingConcentration(self):   
        with open('heatmap.txt', 'w') as file:
            self.current_positions_list.append(self.pheromone_algorithm.get_robot_position())
            file.write("{}\n".format(self.current_positions_list))
            file.flush()

    def main(self): 
        # while robot.supervisor.step(robot.timestep) != -1:
        robot.reset_actuator_values()
        robot.get_sensor_input()
        robot.blink_leds()
        
        self.present_time = robot.supervisor.getTime()
        # self.mappingTrajectory(robot_names)
        # self.mappingConcentration(robot_names)
        
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
        
        self.is_red, self.is_brown = robot.get_camera_input()
        
        if self.robot_states[self.robot_name] == RobotState.SEARCHING:  
            if self.present_time >= 5:
                print("started pheromone")
                #pheromone algorithm works after 5 secs of scattering
                # self.startPheromone(robot_names)
                self.startPheromone()
            else:
                #random walk for less than 5 secs 
                # if wall_detected() or cliff_detected():
                # print("wall in front")
                # print("started random")
                # left_speed = robot.speeds[0]
                # right_speed = robot.speeds[1]
                # robot.left_motor.setVelocity(left_speed)
                # robot.right_motor.setVelocity(right_speed)
                
                print("is brown found", self.is_brown)
                if self.is_brown:
                    # if 460785 < number_brown_pixels < 517140:
                    #         robot.left_motor.setVelocity(0.0)
                    #         robot.right_motor.setVelocity(0.0)
                    #         sys.exit(0)
                    print("started random")
                    left_speed = robot.speeds[0]
                    right_speed = robot.speeds[1]
                    robot.left_motor.setVelocity(left_speed)
                    robot.right_motor.setVelocity(right_speed)
                else:
                    print("started random")
                    left_speed = robot.speeds[0]
                    right_speed = robot.speeds[1]
                    robot.left_motor.setVelocity(left_speed)
                    robot.right_motor.setVelocity(right_speed)

            self.grabbing()
            
        
        elif self.robot_states[self.robot_name] == RobotState.HOMING:
            print("State right now", self.robot_states[self.robot_name] )
            print("going home")   
            # self.homing(robot_names)
            self.homing()
            

        robot.set_actuators()
        robot.step()


if __name__ == "__main__":
    supervisor = Supervisor()
    robot_names = ['e1', 'e2']
    controllers = []

    for robot_name in robot_names:
        if supervisor.getName() == robot_name:
            print("Robot name: ", robot_name)

            robot = InitializeRobot(name=robot_name, supervisor=supervisor)
            pheromone = PheromoneAlgorithm(robot)
            controller = Controller(robot, pheromone, robot_name)
            pheromone.setController(controller)
            controllers.append(Controller(robot, pheromone, robot_name)) 
    print(controllers)
    # robot = InitializeRobot()    
    # pheromone = PheromoneAlgorithm(robot)
    # controller = Controller(robot, pheromone)
    # # experiment = Experiment(robot, pheromone)
    # # experiment.setController(controller)
    # pheromone.setController(controller )
    # controller.main()

    while supervisor.step(robot.timestep) != -1:
        for controller in controllers:
            controller.main()