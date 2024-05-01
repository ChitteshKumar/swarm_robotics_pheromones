"""robot_foraging controller."""
import numpy as np
# from controller import Robot, Motor, DistanceSensor, LED, Node, Camera, Emitter, Receiver, Supervisor
from controller import Node, Supervisor
import sys
import cv2  # OpenCV for image processing
import math
import matplotlib.pyplot as plt 

class RobotState:
    SEARCHING = 1
    HOMING = 2
    IDLE = 3

if __name__ == "__main__":

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
    # MAX_SPEED = 6.28 
    MAX_SPEED = 3.14
    speeds = [0.0, 0.0]
    offsets = [0.5 * MAX_SPEED, 0.5 * MAX_SPEED]
    counter = 0
    coefficients = [[0.942, -0.22], [0.63, -0.1], [0.5, -0.06],  [-0.06, -0.06], [-0.06, -0.06], [-0.06, 0.5], [-0.19, 0.63], [-0.13, 0.942]]

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
             if ground_sensors[i] and ground_sensors[i] < 100.0:
                 return True
         return False
    
    def wall_detected():
        # Check if any distance sensor reading indicates a wall
        for i in range(DISTANCE_SENSORS_NUMBER):
            if distance_sensors_values[i] > 80.0:
                return True  # Wall detected
        return False  # No wall detected
    
    def go_forward():
        passive_wait(0.2)
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
        
    
    def go_backwards():
        passive_wait(0.2)
        left_motor.setVelocity(-MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED)
        
    
    def turn_left():
        passive_wait(0.2)
        left_motor.setVelocity(MAX_SPEED * 0.5) # slower velocity for turning 
        right_motor.setVelocity(MAX_SPEED)
        
        
    def turn_right():
        passive_wait(0.2)
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED * 0.5 )

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
    
    #--------------------------------- Random Foraging ---------------------------------------------------
    senses_food = False
    def grabbing():
        if get_camera_input():
            if 65000 < number_pixels < 517140: #66080
                print("seeing food")
                senses_food = True
            else:
                senses_food = False
        else:
            senses_food = False

        print("FOOD GRABBED", senses_food)
        return senses_food
    
    def get_robot_position():
        robot_node = supervisor.getSelf()
        position = robot_node.getPosition()
        return position
    
    start_position = get_robot_position()  #start [0.0, -0.25, 0.0]

    def distanceToHome(current_position):
        distance_to_start = math.sqrt((start_position[0] - current_position[0])**2 + 
                                  (start_position[1] - current_position[1])**2)
        return distance_to_start
    
    def homing():
        print("start", start_position)
        current_position = get_robot_position()
        print("current position: ", current_position)
        homing_vector = [start_position[i] - current_position[i] for i in range(2)]
        # print(" homing vector: ", homing_vector)
        
        distance_to_start = distanceToHome(current_position)
        print("distance to start :", round(distance_to_start,2))

        #robots direction - direction of the vector from the current position to the target relative to the coordinate system,
        angle_to_start = math.atan2(homing_vector[1], homing_vector[0])
        # print("angle to start(type dont know): ", round(angle_to_start,2))

        #values kept just by testing, adjust this threshold based on accuracy
        if 0.05 <= round(distance_to_start,2) <= 0.06:  
            print("Reached home. Stopping!")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            sys.exit(0) 
        else:
            print("Not yet at home.")
            if round(angle_to_start,2) > 0.1:
                turn_left()
            else:         
                turn_right()
            go_forward()
            
    robot_state = RobotState.SEARCHING
    def updateRobotState():
        global robot_state
        print("state: ", robot_state)
        print("what is grabbing right now: ", grabbing())

        if robot_state == RobotState.SEARCHING:
            print("12345")
            #in searching mode, grab food
            if grabbing():
                #once food is found chnage the state 
                robot_state = RobotState.HOMING
    
    #robot trajectory :
    displayA = supervisor.getDevice("displayA")  
    x_trajectory = []
    y_trajectory = []
    def mappingTrajectory():   
        #plotting graph 
        current_position = get_robot_position()
        x_trajectory.append( current_position[0] )
        y_trajectory.append( current_position[1] )

        plt.plot(x_trajectory,y_trajectory)
        plt.title("ROBOT TRAJECTORY")
        plt.xlabel("X_AXIS")
        plt.ylabel("Y_AXIS")
        plt.grid(True)
        plt.savefig("trajectory_random_foraging.png")

        image = displayA.imageLoad( "trajectory_random_foraging.png" )
        displayA.imagePaste(image, 0,0, False)
        displayA.imageDelete(image)

    robot_pos_list=[]
    left_speed = 0
    right_speed = 0 
    while supervisor.step(timestep) != -1:
        reset_actuator_values()
        get_sensor_input()
        blink_leds()

        mappingTrajectory()

        #random movement of the robot (kind of avoids obstacles)
        for i in range(2):
            speeds[i] = 0.0
            for j in range(8):
                speeds[i] += coefficients[j][i] * (1.0 - (distance_sensors_values[j]/ (1024/2)))
        print("speeds: ",speeds)
        
        print("main loop robot state: ", robot_state)
        updateRobotState()
        if robot_state == RobotState.SEARCHING:    
            if wall_detected() or cliff_detected():
                print("walll in frint")
                left_speed = speeds[0]
                right_speed = speeds[1]
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                # print("Left side speed: ", left_motor.getVelocity(), "right side speed: ", right_motor.getVelocity())
            grabbing()
        
        elif robot_state == RobotState.HOMING:
            print("going home")   
            homing()

        set_actuators()
        step()


        