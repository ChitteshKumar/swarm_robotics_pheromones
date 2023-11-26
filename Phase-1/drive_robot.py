"""drive_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor



if __name__ == "__main__":
    
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 6.28 #angular velocity
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    left_motor = robot.getMotor('motor_2')
    right_motor = robot.getMotor('motor_1')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    #drive the robot in polygon shape
    num_side = 4
    length_side = 0.25
    wheel_radius = 0.025
    
    linear_vel = wheel_radius * max_speed
    
    time_straight = length_side/linear_vel
    
    start_time = robot.getTime()
    
    angle_rotation = 6.28 / num_side
    dist_wheels = 0.090
    rate_rotation = (2*linear_vel)/dist_wheels
    duration_turn = angle_rotation/rate_rotation
    
    rotation_time_start = start_time + time_straight 
    rotation_time_stop = rotation_time_start + duration_turn
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    # THIS LOOPS READS: 1. sensor values, 2. process data, 3.make decisions, 4.send motor the commands
    while robot.step(timestep) != -1:
        
        current_time = robot.getTime() 
        
        left_speed = max_speed
        right_speed = max_speed
        
        if rotation_time_start < current_time < rotation_time_stop:
            left_speed = -max_speed
            right_speed = max_speed
        elif current_time > rotation_time_stop:
            rotation_time_start = current_time + time_straight       
            rotation_time_stop = rotation_time_start + duration_turn

        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        
    
    # Enter here exit cleanup code.
    