"""food_detection_avoid_obstacles controller."""
import numpy as np
from controller import Robot, Motor, DistanceSensor, LED, Node, Camera, Emitter, Receiver, Supervisor
import sys
import cv2  # OpenCV for image processing
import math

if __name__ == "__main__":
    # create the Robot instance.
    # robot = Robot()
    supervisor = Supervisor()

    #time step of the world
    timestep = int(supervisor.getBasicTimeStep())
    
    #Distance Sensor in the robot
    DISTANCE_SENSORS_NUMBER = 8
    distance_sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
    distance_sensors =  [None] * DISTANCE_SENSORS_NUMBER
    distance_sensors_values = [0.0] * DISTANCE_SENSORS_NUMBER  # Initialize distance_sensors_values 
    for i, name in enumerate(distance_sensor_names):
        distance_sensors[i] = supervisor.getDevice(name)
        distance_sensors[i].enable(timestep)

    #Ground Sensor in the device
    GROUND_SENSORS_NUMBER = 3
    ground_sensor_names = ["gs0", "gs1", "gs2"]
    # Silently initialize the ground sensors if they exist
    ground_sensors = [None] * GROUND_SENSORS_NUMBER
    ground_sensors_values = [0.0] * GROUND_SENSORS_NUMBER

    devices_number = supervisor.getNumberOfDevices()
    for i in range(devices_number):
        dtag = supervisor.getDeviceByIndex(i)
        dname = dtag.getName()
        dtype = dtag.getNodeType()
        if dtype == Node.DISTANCE_SENSOR and len(dname) == 3 and dname[0] == 'g' and dname[1] == 's':
            id = int(dname[2])
            if 0 <= id < GROUND_SENSORS_NUMBER:
                ground_sensors[id] = supervisor.getDevice(ground_sensor_names[id])
                ground_sensors[id].enable(timestep)
    
    #Led in the device
    LEDS_NUMBER = 10
    led_names = ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"]
    leds = [None] * LEDS_NUMBER
    leds_values = [False] * LEDS_NUMBER  # Initialize leds_values
    for i, name in enumerate(led_names):
        leds[i] = supervisor.getDevice(name)
    
    #sensor for camera
    CAMERA_NAME = "camera"
    camera = supervisor.getDevice(CAMERA_NAME)
    camera.enable(timestep)
    
    # Define emitter and receiver
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(timestep)
    

    #Initialising the motors 
    left_motor = supervisor.getDevice("left wheel motor")
    right_motor = supervisor.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    #About the robot
    MAX_SPEED = 6.28 
    speeds = [0.0, 0.0]
    offsets = [0.5 * MAX_SPEED, 0.5 * MAX_SPEED]
    counter = 0
    
    # Breitenberg-Principle Weights
    wall_avoidance_weights = [[-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
               [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
    obstacles_avoidance_weights = [[1.0, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
               [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
    incoming_avoidance_weights = [[-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
               [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
    
    def step():
        if supervisor.step(timestep) == -1:
            supervisor.cleanup()
            sys.exit(0)
    
    def passive_wait(sec):
        start_time = supervisor.getTime()
        while start_time + sec > supervisor.getTime():
            step()
    
    def reset_actuator_values():
        for i in range(2):
            speeds[i] = 0.0
        for i in range(LEDS_NUMBER):
            # leds[i].setValue(False)
            leds[i].set(0)
    
    def get_sensor_input():
        for i, sensor in enumerate(distance_sensors):
            distance_sensors_values[i] = sensor.getValue()
            # Scale the data to have a value between 0.0 and 1.0
            # 1.0 representing something to avoid, 0.0 representing nothing to avoid
            distance_sensors_values[i] /= 4096
        
        for i, sensor in enumerate(ground_sensors):
            if sensor:
                ground_sensors_values[i] = sensor.getValue()

    def set_actuators():
        for i, led in enumerate(leds):
            if leds_values[i]:
                led.set(1)
            else:
                led.set(0)
        left_motor.setVelocity(speeds[0])
        right_motor.setVelocity(speeds[1])
    
    def blink_leds():
        global counter
        counter += 1
        leds_values[(counter // 10) % LEDS_NUMBER] = True

    def cliff_detected():
         for i in range(GROUND_SENSORS_NUMBER):
             if ground_sensors[i] and ground_sensors[i] < 500.0:
                 return True
         return False
    
    def wall_detected():
        # Check if any distance sensor reading indicates a wall
        for i in range(DISTANCE_SENSORS_NUMBER):
            if distance_sensors_values[i] < 500.0:
                return True  # Wall detected
        return False  # No wall detected
    
    #--------------------------------- Collsion mechanism to be improved from the E-puck line --------------------------------- 
    def run_braitenberg():
        left_speed = 0.0
        right_speed = 0.0
        
        if cliff_detected() or wall_detected():
            for i in range(2):
                speeds[i] = 0.0
                for j in range(DISTANCE_SENSORS_NUMBER):
                    speeds[i] += distance_sensors_values[j] * wall_avoidance_weights[j][i]
                speeds[i] = offsets[i] + speeds[i] * MAX_SPEED
                speeds[i] = min(MAX_SPEED, max(-MAX_SPEED, speeds[i]))
            left_speed = speeds[0]
            right_speed = speeds[1]
            
        else:
            for i in range(2):
                speeds[i] = 0.0
                
                # Robot/Obstacle avoidance behavior
                for j in range(DISTANCE_SENSORS_NUMBER):
                    speeds[i] += distance_sensors_values[j] * obstacles_avoidance_weights[j][i]
           
                # Additional logic for detecting other robots or obstacles
                for k in range(GROUND_SENSORS_NUMBER):
                    if ground_sensors_values[k] > 150.0:
                        # #Adjust weights based on the presence of incoming robots
                        speeds[i] += incoming_avoidance_weights[k][i]
                
                speeds[i] = offsets[i] + speeds[i] * MAX_SPEED
                speeds[i] = min(MAX_SPEED, max(-MAX_SPEED, speeds[i]))
            left_speed = speeds[0]
            right_speed = speeds[1]   

        #setting the motor speed according to the situation    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed) 
    #--------------------------------------------------------------------------------------------- 
      
    def go_forward():
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
        passive_wait(0.2)
    
    def go_backwards():
        left_motor.setVelocity(-MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED)
        passive_wait(0.2)
    
    def turn_left():
        left_motor.setVelocity(-MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
        passive_wait(0.2)
        
    def turn_right():
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED)
        passive_wait(0.2)
   
    def get_camera_input():
        global number_pixels
        # Get the colors perceived by the camera
        image = camera.getImage()
        # Assuming the camera resolution is 128x128 pixels
        width, height = camera.getWidth(), camera.getHeight()
        # Convert image to a format suitable for OpenCV
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
  
    def send_message(message):
        emitter.send(message)

    def receive_message():
        if receiver.getQueueLength() > 0:
            message = receiver.getString()
            receiver.nextPacket()
            return message
        return None 
    #--------------------------------- PHEROMONE---Algorithm --------------------------------- 
    def get_robot_position():
        robot_node = supervisor.getSelf()
        position = robot_node.getPosition()
        return position
    
    # pheromone_grid = [[0.0]*10 for _ in range(10)]
    pheromone_grids = {}

    #define pheromone properties
    intensity = 1.0
    evaporation_rate = 0.1
    # sensing_range = 0.5

    #STEP-1
    def deposit_pheromone(robot_name, robot_pos):
        if robot_name not in pheromone_grids:
            pheromone_grids[robot_name] = [[0.0]*10 for _ in range(10)]

        #deposit pheromones 
        pheromone_grids[robot_name][int(robot_pos[0]*10)][int(robot_pos[1]*10)] += intensity
        print(f"deposited at: {robot_pos[0]}, {robot_pos[1]}" )
        print("GRID (before sensing): ", pheromone_grids[robot_name])

    #STEP-2
    def sense_pheromone(robot_name, robot_pos):
        #esuring each robot's grid is added to the dictionary 
        if robot_name not in pheromone_grids:
            pheromone_grids[robot_name] = [[0.0]*10 for _ in range(10)]

        #pheromone at current position 
        current_sensed_pheromone = pheromone_grids[robot_name][int(robot_pos[0]*10)][int(robot_pos[1]*10)] 

        # neighbors = [
        #     (self.x, self.y + 1),  # Up
        #     (self.x, self.y - 1),  # Down
        #     (self.x - 1, self.y),  # Left
        #     (self.x + 1, self.y),  # Right
        # ]

        # pheromones_around = {}

        # for neighbor_x, neighbor_y in neighbors:
        
        neighbour_sensed_pheromone = []
        #calculate the pheromone concentration at neighbouring cells (up, down left, right -> the list is in this order)
        for i,j in [(-1,0), (1,0), (0,-1), (0,1)]:
            new_pos_x = int(robot_pos[0]*10) + i 
            new_pos_y = int(robot_pos[1]*10) + j
            if 0 <= new_pos_x < 10 and 0<= new_pos_y < 10:
                # neighbour_sensed_pheromone += pheromone_grids[robot_name][new_pos_x][new_pos_y]
                neighbour_sensed_pheromone.append(pheromone_grids[robot_name][new_pos_x][new_pos_y])

        pheromone_grids[robot_name][int(robot_pos[0]*10)][int(robot_pos[1]*10)] += 1.0

        sensed_pheromone = [current_sensed_pheromone] + neighbour_sensed_pheromone


        print(f"GRID (after sensing) of {robot_name}: ", pheromone_grids[robot_name])        
        # print(f"Current pheromone of {robot_name} : ", current_sensed_pheromone)
        # print("neighbour : ", neighbour_sensed_pheromone)
        print("combined (current + neighbour): ", sensed_pheromone)
        
        adjust_robot_behavior(sensed_pheromone, robot_name) #not working 
        return sensed_pheromone
    """
    # Sense pheromones (simplified example)
    # Modify this part based on your robot's behavior
    # For example, check for pheromones in the sensing range
    sensed_pheromones = sense_pheromones(robot_position)
    if sensed_pheromones:
        adjust_robot_behavior(sensed_pheromones)  # Implement this function
    """
    #STEP-3
    def adjust_robot_behavior(sensed_pheromone, robot_name):
        # Threshold to determine whether pheromones are present
        threshold = 0.5
        if max(sensed_pheromone) > threshold:
            # Perform some behavior when pheromones are detected
            print(f"Pheromones detected for {robot_name}, adjusting behavior...")
            
            #find the postion of the higher value of pheromone 
            max_pheromone = sensed_pheromone.index(max(sensed_pheromone))
            # print("MAX PHEROMONE DETECTED at index: ", max_pheromone)
        
            if max_pheromone == 0: #pheromone is highest in its current position 
                print("this is highest, no change needed")
                run_braitenberg()
            elif max_pheromone == 1: #pheromone is highest in upper position 
                print("this is highest, in upper direction")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
            elif  max_pheromone == 2: #pheromone is highest in down position 
                print("this is highest, in opposite (down) direction")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
            elif max_pheromone == 3: #pheromone is highest in left position 
                print("this is highest, in left direction")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
            elif max_pheromone ==4: #pheromone is highest in right position 
                print("this is highest, in right direction")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
     
            

            # else:
            #     # Perform default behavior when no pheromones are detected
            #     print(f"No pheromones detected for {robot_name}, continuing default behavior...")
            #     # For example, continue with Braitenberg algorithm
            #     run_braitenberg()

    # def position_max_pheromone(robot_pos, sensed_pheromone):
    #     #get the direction of high valued pheromone
    #     x = int(robot_pos[0]*10)
    #     y = int(robot_pos[1]*10)
    #     neighbours = {
    #         'up' : sensed_pheromone[0],
    #         'down' : sensed_pheromone[1],
    #         'left' : sensed_pheromone[2],
    #         'right' : sensed_pheromone[3]
    #     }
    #     max_direction = max(neighbours, key=neighbours.get)
    #     return max_direction
    
    #STEP-4
    def evaporate_pheromone(robot_name, elapsed_time, robot_pos):
        #decay factor 
        decay_factor = math.exp(-evaporation_rate * elapsed_time)
        pheromone_grids[robot_name][int(robot_pos[0])*10][int(robot_pos[1])*10] *= decay_factor

    #track time for evaporation
    previous_time_evp = timestep
    evaporation_interval = 2.0 # in seconds 
    #---------------------------------- End of Ph.Algorithm --------------------------------- 
    
    counter = 0
    while supervisor.step(timestep) != -1:
        reset_actuator_values()
        get_sensor_input()
        blink_leds()
        
        # Receive messages from other robots
        received_message = receive_message()
        robot_names = ['e1', 'e2', 'e3']

        for robot_name in robot_names:
            if supervisor.getName() == robot_name:

                #position of the robot
                robot_pos = get_robot_position()
                print(f"Current positon of {robot_name}: {robot_pos}")

                if cliff_detected():
                    go_backwards()
                    turn_left()
                else:
                    run_braitenberg()
                    
                    if get_camera_input():
                        # the code below is the instructions for e1 robot to stop 
                        if 66080 < number_pixels < 517140:
                            # Stop the robot if a red object is detected
                            speeds = [0.0, 0.0]
                            set_actuators()
                            print("Red object detected, stopping.")
                            # Send a message to other robots to stop
                            send_message("STOP")
                            # robot stops but keep the simulation running,
                            left_motor.setVelocity(0)
                            right_motor.setVelocity(0)
                            # sys.exit(0)      
                            continue

                    #deposition of pheromone 
                    deposition = deposit_pheromone(robot_name, robot_pos)
                    #sense the pheromone 
                    sensing = sense_pheromone(robot_name, robot_pos)         
                    #the two functions are after red object detection since we do not want to deposit and sense extra pheromones which are not useful
        
        # current_time = timestep
        # elapsed_time = current_time - previous_time_evp 

        # if elapsed_time > evaporation_interval:
        #     #evaporation of the pheromone and update the previous_time_evp  time 
        #     evaporate_pheromone(elapsed_time, robot_name)
        #     previous_time_evp = current_time


        set_actuators()
        step()