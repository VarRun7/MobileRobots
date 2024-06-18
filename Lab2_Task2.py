# Path: /Users/varennyamjoshi/FAIRIS-Lite/WebotsSim/controllers/Lab2_Task2.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
os.chdir("../..")

# Import necessary modules
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position (waypoint P0) listed in maze file
robot.move_to_start()

# Define PID parameters
Kp_front = 0.5  # Proportional gain for front sensor
Kp_side = 2.65   # Proportional gain for side sensors
desired_distance = 0.5  # Desired distance from the front wall
desired_side_distance = 0.3  # Desired minimum distance from side walls
#desired_stop_distance = 0.5  # Desired distance to stop from the front wall

# Choose the left or right wall
wall_choice = 'right'

# Convert the input to lowercase for case-insensitive comparison
wall_choice = wall_choice.lower()

# Check if the user input is valid
if wall_choice not in ['left', 'right']:
    print("Invalid choice. Please enter 'left' or 'right'.")
    exit()

# Use the user's choice in your code
if wall_choice == 'right':
    # Implement wall-following behavior for the left wall
    print("Wall-following behavior for the right wall")
    # Main loop:
    while robot.experiment_supervisor.step(robot.timestep) != -1:

        # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
        print("Lidar Right Distance Reading (m):", robot.get_lidar_range_image()[600])
        print("Lidar Left Distance Reading (m):", robot.get_lidar_range_image()[200])
        print(" ")
        distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
        distance_front_right_wheel_traveled = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
        distance = (distance_front_left_wheel_traveled + distance_front_right_wheel_traveled)/2
    
        print("Time", robot.experiment_supervisor.getTime())
        print("Total Distance Covered (m):", distance)
        print(" ")
        
        # Read the sensors:
        front_sensor = robot.get_lidar_range_image()[400]
        left_sensor = robot.get_lidar_range_image()[200]
        right_sensor = robot.get_lidar_range_image()[600]
    
        # Process sensor data and apply PID control
        # Front sensor PID control
        front_error = desired_distance - front_sensor  # Calculate error
        front_correction = Kp_front * front_error       # Calculate correction

        # Side sensors PID control
        side_error = desired_side_distance - min(left_sensor, right_sensor)  # Calculate error
        side_correction = Kp_side * side_error           # Calculate correction
        time = robot.experiment_supervisor.getTime()
        
        # Apply corrections to motor velocities or positions    
        if left_sensor <= 0.5:
           robot.set_right_motors_velocity(10 + side_correction)
           robot.set_left_motors_velocity(10 - side_correction)
            
        if right_sensor <= 0.5:
           robot.set_right_motors_velocity(10 - side_correction)
           robot.set_left_motors_velocity(10 + side_correction)
            
        direction = robot.get_compass_reading()
        print("direction", direction)
        # Print sensor readings
        print(" ")
        print("Lidar Front Distance Reading (m):", robot.get_lidar_range_image()[400])
        print("Lidar Right Distance Reading (m):", robot.get_lidar_range_image()[600])
        #print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
        print("Lidar Left Distance Reading (m):", robot.get_lidar_range_image()[200])
        print(" ")

        # Exit condition if reached
        if right_sensor >= 0.8:
            #while direction < direction+90:
                robot.set_right_motors_velocity(8)
                robot.set_left_motors_velocity(15)
            
        if front_sensor <= 0.75:
           robot.set_right_motors_velocity(15)
           robot.set_left_motors_velocity(3)    

    
    
else:
    # Implement wall-following behavior for the left wall
    print("Wall-following behavior for the left wall")
  # Main loop:
    while robot.experiment_supervisor.step(robot.timestep) != -1:

        # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
        #print("Lidar Front Distance Reading (m):", robot.get_lidar_range_image()[400])
        print("Lidar Right Distance Reading (m):", robot.get_lidar_range_image()[600])
        print("Lidar Left Distance Reading (m):", robot.get_lidar_range_image()[200])
        print(" ")
        distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
        distance_front_right_wheel_traveled = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
        distance = (distance_front_left_wheel_traveled + distance_front_right_wheel_traveled)/2
    
        print("Time", robot.experiment_supervisor.getTime())
        print("Total Distance Covered (m):", distance)
        print(" ")
        
        # Read the sensors:
        front_sensor = robot.get_lidar_range_image()[400]
        left_sensor = robot.get_lidar_range_image()[200]
        right_sensor = robot.get_lidar_range_image()[600]
    
        # Process sensor data and apply PID control
        # Front sensor PID control
        front_error = desired_distance - front_sensor  # Calculate error
        front_correction = Kp_front * front_error       # Calculate correction

        # Side sensors PID control
        side_error = desired_side_distance - min(left_sensor, right_sensor)  # Calculate error
        side_correction = Kp_side * side_error           # Calculate correction
        time = robot.experiment_supervisor.getTime()
        
        # Apply corrections to motor velocities or positions    
        if left_sensor <= 0.5:
           robot.set_right_motors_velocity(10 + side_correction)
           robot.set_left_motors_velocity(10 - side_correction)
            
        if right_sensor <= 0.5:
           robot.set_right_motors_velocity(10 - side_correction)
           robot.set_left_motors_velocity(10 + side_correction)
            
        direction = robot.get_compass_reading()
        print("direction", direction)
        # Print sensor readings
        print(" ")
        print("Lidar Front Distance Reading (m):", robot.get_lidar_range_image()[400])
        print("Lidar Right Distance Reading (m):", robot.get_lidar_range_image()[600])
        print("Lidar Left Distance Reading (m):", robot.get_lidar_range_image()[200])
        print(" ")

        # Exit condition if reached
        if left_sensor >= 0.8:
            #while direction < direction+90:
                robot.set_right_motors_velocity(15)
                robot.set_left_motors_velocity(8)
            
        if front_sensor <= 0.75:
           robot.set_right_motors_velocity(3)
           robot.set_left_motors_velocity(15)  


# Enter here exit cleanup code.
