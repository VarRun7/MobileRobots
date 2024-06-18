# Path: /Users/varennyamjoshi/FAIRIS-Lite/WebotsSim/controllers/Lab3_Task2.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
import random
import time
from WebotsSim.libraries.MyRobot import MyRobot

# Change working directory to the root of FAIRIS-Lite
os.chdir("../..")

# Create the robot instance
robot = MyRobot()

# Load the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab3/Lab3_Task2_2.xml'
robot.load_environment(maze_file)

# Move robot to the start point "S"
robot.move_to_start()

# Goal position
goal_position = None  # Set to the actual goal position when detected

Kp_distance = 3.0 # for motion to goal
Kp_orientation = 0.1 #for turning
# Maximum distance from obstacles
max_distance_from_obstacle = 0.5 # to stop before the object
# PID controller parameters
wall_to_follow = 'R' #R for right and L for left
D_maintain=0.5 #maintain distance from wall
D_min=0.43 #minimum distance for wall following
Kp=6 #forward PID
max_v = 20 #max velocity




def forward_saturation(v):
    #max_v =20
    if v >= max_v:
        v = max_v
        return v
    elif v <= -max_v:
        v = -max_v
        return v
    else:
        return v

def forward_PID(D_maintain, Kp):
    # Replace this with actual lidar measurements
    # Minimum lidar measurement in range of front measurements
    f_d = min(robot.get_lidar_range_image()[350:450])
    error = f_d - D_maintain
    return forward_saturation(Kp * error)

def wall_follow_PID(D_min, Kp, wall_to_follow):
    V_f = forward_PID(D_maintain, Kp)
    
    # Replace these with actual lidar measurements
    rd = min(robot.get_lidar_range_image()[550:650]) 
    ld = min(robot.get_lidar_range_image()[150:250])
    print('rd', rd)
    
    print('ld', ld)
    
     
    if wall_to_follow == 'R':
        error = D_min - rd
        if rd < D_min:
            V_r = forward_saturation(V_f)
            V_l = forward_saturation(V_f - abs(Kp * error))
        elif rd > D_min:
            V_r = forward_saturation(V_f - abs(Kp * error))
            V_l = forward_saturation(V_f)
        elif ld < 0.75:
            error = 0.75 - ld
            V_r = forward_saturation(V_f - abs(Kp * error))
            V_l = forward_saturation(0)
        else:
            V_l = V_f
            V_r = V_f
    else:
        error = D_min - ld
        if ld < D_min:
            V_r = forward_saturation(V_f - abs(Kp * error))
            V_l = forward_saturation(V_f)
        elif ld > D_min:
            V_r = forward_saturation(V_f)
            V_l = forward_saturation(V_f - abs(Kp * error))
        elif rd < 0.75:
            error = 0.75 - rd
            V_r = forward_saturation(0)
            V_l = forward_saturation(V_f - abs(Kp * error))
        else:
            V_l = V_f
            V_r = V_f
    print('vl, vr', V_l, V_r)
    return V_l, V_r

# Define rotate function
def rotate(degrees_to_rotate):
    # Introduce error of ±5 degrees
    degrees_to_rotate += random.uniform(-5, 5)
    
    # Ensure that the degrees_to_rotate is within the range of 0 to 360 degrees
    degrees_to_rotate %= 360
    
    # Assume there's a function to get current compass heading called get_compass_reading()
    current_reading = robot.get_compass_reading()
    target_reading = (current_reading + degrees_to_rotate) % 360
    
    # Set left and right motor velocities for rotation
    robot.set_left_motors_velocity(-10)
    robot.set_right_motors_velocity(10)
    
    # Rotate until the robot reaches the target_reading within an error of 10 degrees
    while True:
        current_heading = robot.get_compass_reading()
        error = (target_reading - current_heading) % 360  # Calculate the error in degrees
        if error <= 10 or error >= 350:  # Check if the error is within ±10 degrees or wrapping around
            break
        # Continue rotating if the error is greater than 10 degrees
        # Adjust the rotation direction based on the shortest path to the target
        if error > 180:
            robot.set_left_motors_velocity(10)
            robot.set_right_motors_velocity(-10)
        else:
            robot.set_left_motors_velocity(-10)
            robot.set_right_motors_velocity(10)
            
    # Stop the robot after reaching the target_heading
    #robot.set_left_motors_velocity(0)
    #robot.set_right_motors_velocity(0)
    
    return target_reading
    
# Main loop
while robot.experiment_supervisor.step(robot.timestep) != -1:
    # Get recognized objects from the camera
    # Look for the yellow cylinder (goal)
    front_sensor = robot.get_lidar_range_image()[400]
    left_sensor = robot.get_lidar_range_image()[200]
    right_sensor = robot.get_lidar_range_image()[600]
    goal = None
    objects = robot.rgb_camera.getRecognitionObjects()

    if (objects != None) & (front_sensor > 0.5):
        for obj in objects:
            id = obj.getId()
            position = obj.getPosition()
            orientation = obj.getOrientation()
            size = obj.getSize()
            position_on_image = obj.getPositionOnImage()  # Assuming getPositionOnImage() returns a reference to an array
        
            # Dereference the array and store its values in position_on_image_list
            position_on_image_list = [value for value in position_on_image]
        
            # Now position_on_image_list contains the actual values of the array
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
        if ((front_sensor < 0.6 ) and wall_to_follow == 'R'):
            start_time = time.time()
            print('--------------------')
            robot.set_left_motors_velocity(2)
            robot.set_right_motors_velocity(-2)  
            while True:
                if time.time() - start_time >= 0.5:
                    robot.set_left_motors_velocity(2)
                    robot.set_right_motors_velocity(2)  
                    print("......................................")
                    break
        Vl, Vr = wall_follow_PID(D_min, Kp, wall_to_follow)  # Provide appropriate arguments
        robot.set_left_motors_velocity(Vl)
        robot.set_right_motors_velocity(Vr)
        
        f_d = min(robot.get_lidar_range_image()[350:450])  # Replace with actual lidar reading
        rd = min(robot.get_lidar_range_image()[550:650]) 
        ld = min(robot.get_lidar_range_image()[150:250])  
        print('f_d', f_d)

        # Provide the wall to follow
       
            #rotate(90)
                # include in the function. 
        if robot.get_lidar_range_image()[600] > 0.8:
            robot.set_left_motors_velocity(8)
            robot.set_right_motors_velocity(3)
        else:
            Vl, Vr = wall_follow_PID(D_min, Kp, wall_to_follow) 
            robot.set_left_motors_velocity(Vl)
            robot.set_right_motors_velocity(Vr)
                
      '''  #elif f_d < 0.25 and wall_to_follow == 'L':
        #  rotate(-45)
        # while robot.get_compass_reading() != target_reading:
        #    rotate(-45)

        #rotate(90)def rotate(degrees_to_rotate):
    # Assume there's a function to get current compass heading called get_compass_heading()
    start_time = time.time()
    current_reading = robot.get_compass_reading()
    robot.set_left_motors_velocity(10)
    robot.set_right_motors_velocity(-10)
    
    # Calculate the target heading after rotation
    # global 
    target_reading = (current_reading + degrees_to_rotate) % 360
    while True:
        if time.time() - start_time >= 0:
            print("......................................")
            break
        return target_reading
    # while loop here 
    # Assume there's a function to rotate the robot to a specific heading called rotate_to_heading()
    #rotate_to_heading(target_heading)
'''
