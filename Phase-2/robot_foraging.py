"""food_detection_avoid_obstacles controller."""
import numpy as np
from controller import Robot, Motor, DistanceSensor, LED, Node, Camera, Emitter, Receiver
import sys
import cv2  # OpenCV for image processing

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    #Sensor in the device
    DISTANCE_SENSORS_NUMBER = 8
    distance_sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
    
    GROUND_SENSORS_NUMBER = 3
    ground_sensor_names = ["gs0", "gs1", "gs2"]
    
    LEDS_NUMBER = 10
    led_names = ["led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"]
    
    #sensor for camera
    CAMERA_NAME = "camera"
    camera = robot.getDevice(CAMERA_NAME)
    camera.enable(int(robot.getBasicTimeStep()))
    
    # Define emitter and receiver
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(int(robot.getBasicTimeStep()))
    
    distance_sensors =  [None] * DISTANCE_SENSORS_NUMBER
    for i, name in enumerate(distance_sensor_names):
        distance_sensors[i] = robot.getDevice(name)
        distance_sensors[i].enable(int(robot.getBasicTimeStep()))
    
    leds = [None] * LEDS_NUMBER
    for i, name in enumerate(led_names):
        leds[i] = robot.getDevice(name)
    
    devices_number = robot.getNumberOfDevices()
    for i in range(devices_number):
        dtag = robot.getDeviceByIndex(i)
        dname = dtag.getName()
        dtype = dtag.getNodeType()
        if dtype == Node.DISTANCE_SENSOR and len(dname) == 3 and dname[0] == 'g' and dname[1] == 's':
            id = int(dname[2])
            if 0 <= id < GROUND_SENSORS_NUMBER:
                ground_sensors[id] = robot.getDevice(ground_sensor_names[id])
                ground_sensors[id].enable(int(robot.getBasicTimeStep()))
    
    # ground_sensors = [None] * GROUND_SENSORS_NUMBER
    # Initialize ground sensors (if they exist)
    # for i,name in enumerate(ground_sensor_names):
                # device = robot.getDevice(name)
                # if device:
                    # ground_sensors[i] = device
                    # ground_sensors[i].enable(int(robot.getBasicTimeStep()))
    
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    MAX_SPEED = 6.28 
    speeds = [0.0, 0.0]
    # if an object is detected using the camera
    object_found = False
    
    # Breitenberg stuff
    weights = [[-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
               [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
               
    offsets = [0.5 * MAX_SPEED, 0.5 * MAX_SPEED]
    
    distance_sensors_values = [0.0] * DISTANCE_SENSORS_NUMBER  # Initialize distance_sensors_values 
    # Silently initialize the ground sensors if they exist
    ground_sensors = [None] * GROUND_SENSORS_NUMBER
    ground_sensors_values = [0.0] * GROUND_SENSORS_NUMBER
    leds_values = [False] * LEDS_NUMBER  # Initialize leds_values
    
    def step():
        if robot.step(int(robot.getBasicTimeStep())) == -1:
            robot.cleanup()
            sys.exit(0)
    
    def passive_wait(sec):
        start_time = robot.getTime()
        while start_time + sec > robot.getTime():
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
    
    def cliff_detected():
        # for i, sensor_value in enumerate(ground_sensors_values):
            # if not ground_sensors[i]:
                # return False
            # if sensor_value < 500.0:
                # return True
        # return False
         for i in range(GROUND_SENSORS_NUMBER):
             if ground_sensors[i] and ground_sensors[i] < 500.0:
                 return True
         return False
    
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
    
    def run_braitenberg():
        global object_found
        
        for i in range(2):
            speeds[i] = 0.0
            for j in range(DISTANCE_SENSORS_NUMBER):
                speeds[i] += distance_sensors_values[j] * weights[j][i]
    
            speeds[i] = offsets[i] + speeds[i] * MAX_SPEED
            speeds[i] = min(MAX_SPEED, max(-MAX_SPEED, speeds[i]))
            
             
    
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
        print("Red detection:", is_red)
        print("Number of red pixels:", number_pixels)
        # if number_pixels > 517140:
            # step()
        return is_red
        
        # Bitwise-AND mask and original image
        # red_detection = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Check if red pixels are present in the image
        # if np.any(red_detection):
            # return True
        # return False
        
        
        """
         # Reshape the data to a 3D array (RGB format)
        camera_data = [camera_data[i:i + 3] for i in range(0, len(camera_data), 3)]
        camera_data = [camera_data[i:i + width] for i in range(0, len(camera_data), width)]
    
        # Calculate the center pixel
        if width > 0 and height > 0:
            center_pixel = camera_data[width // 2][height // 2]
     
    
        # Check if the center pixel corresponds to the color green
        # is_green = center_pixel[0] < 50 and center_pixel[1] > 200 and center_pixel[2] < 50
        # Check if the center pixel corresponds to the color red
        is_red = center_pixel[0] > 200 and center_pixel[1] < 50 and center_pixel[2] < 50
    
        return is_red  """
        
    
    def send_message(message):
        emitter.send(message)

    def receive_message():
        if receiver.getQueueLength() > 0:
            message = receiver.getString()
            receiver.nextPacket()
            return message
        return None
    
    # Main loop
    counter = 0
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        reset_actuator_values()
        get_sensor_input()
        # get_camera_input()
        blink_leds()
        
        # Receive messages from other robots
        received_message = receive_message()
        if get_camera_input():
            # the code below is the instructions for e1 robot to stop 
            if number_pixels > 106080 and number_pixels < 517140:
                # Stop the robot if a red object is detected
                speeds = [0.0, 0.0]
                set_actuators()
                print("Red object detected, stopping.")
                # Send a message to other robots to stop
                send_message("STOP")
                sys.exit(0)
                
        # the code below is the instructions for e2 robot to stop       
        if received_message == "STOP":
            # Stop the robot if a stop message is received
            speeds = [0.0, 0.0]
            set_actuators()
            print("Received stop message, stopping.")
            sys.exit(0)
            
        if cliff_detected():
            go_backwards()
            turn_left()
            # object_found = False # Reset state when avoiding obstacles 
        else:
            run_braitenberg()
        set_actuators()
        step()
    
    
    