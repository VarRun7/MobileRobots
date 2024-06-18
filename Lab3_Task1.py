
# Path: /Users/varennyamjoshi/FAIRIS-Lite/WebotsSim/controllers/Lab3_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
from WebotsSim.libraries.MyRobot import MyRobot

# Change working directory to the root of FAIRIS-Lite
os.chdir("../..")

# Create the robot instance
robot = MyRobot()

# Load the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab3/Lab3_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random starting position (waypoint P0) listed in maze file
robot.move_to_start()

# PID controller parameters
Kp_distance = 3.0
Kp_orientation = 0.1

# Main loop
while robot.experiment_supervisor.step(robot.timestep) != -1:
    # Get recognized objects from the camera
    objects = robot.rgb_camera.getRecognitionObjects()
    
    # Look for the yellow cylinder (goal)
    goal = None
    for obj in objects:
        id = obj.getId()
        position = obj.getPosition()
        orientation = obj.getOrientation()
        size = obj.getSize()
        position_on_image = obj.getPositionOnImage()  # Assuming getPositionOnImage() returns a reference to an array
    
        # Dereference the array and store its values in position_on_image_list
        position_on_image_list = [value for value in position_on_image]
    
        # position_on_image_list contains the actual values of the array
        print("Object ID:", id)
        print("Position:", position)
        print("Orientation:", orientation)
        print("Position on Image:", position_on_image_list)
        
        align = position_on_image_list[0]
        print("Align:", align)
            #if align - 360 <= 10
        # Check if the colors match the desired color [1, 1, 0]
        if align - 360 <= 10:
            goal = obj
            break

    
    if goal:
        # Get goal position and orientation
        goal_position = goal.getPosition()
        goal_orientation = goal.getOrientation()
        
        # Calculate distance to the goal
        distance = math.sqrt(goal_position[0]**2 + goal_position[1]**2)
        
        # Calculate orientation to the goal
        orientation = math.atan2(goal_position[1], goal_position[0])
        
        # Calculate error for distance and orientation
        distance_error = distance - 0.5  # desired distance is 0.5 meters
        orientation_error = orientation
        
        # Calculate wheel velocities using PID controller
        left_velocity = Kp_distance * distance_error - Kp_orientation * orientation_error
        right_velocity = Kp_distance * distance_error + Kp_orientation * orientation_error
        
        # Set wheel velocities
        robot.set_left_motors_velocity(left_velocity)
        robot.set_right_motors_velocity(right_velocity)
        
        # Print debug information
        print("Distance to goal:", distance)
        print("Orientation to goal:", math.degrees(orientation))
    else:
        # If the goal is not visible, rotate until it is in view
        robot.set_left_motors_velocity(1)
        robot.set_right_motors_velocity(-1)
