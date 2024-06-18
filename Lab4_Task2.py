# Varennyam Joshi (U68760211)
# Task 2: Wall Localization
# Path: /Users/varennyamjoshi/FAIRIS-Lite/WebotsSim/controllers/Lab4_Task2.py

import os
import math
import time
from WebotsSim.libraries.MyRobot import MyRobot

# Change working directory to the root of FAIRIS-Lite
os.chdir("../..")

# Create the robot instance
robot = MyRobot()

# Load the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab4/Lab4_Task2_1.xml'
robot.load_environment(maze_file)

landmarks = {
    'yellow': (-2, 2),
    'red': (2, 2),
    'blue': (2, -2),
    'green': (-2, -2)
}

# Move robot to a random starting position
robot.move_to_start()
start_pos = robot.starting_position
x = start_pos.x
y = start_pos.y
theta = start_pos.theta  
print(f"Starting position: x={x}, y={y}, theta={theta}")

# Assuming the grid size and the total number of cells are known
grid_size = (4, 4)  # Example grid size, adjust as per actual environment
visited_cells = [['.' for _ in range(grid_size[0])] for _ in range(grid_size[1])]
current_cell = []

def cot(angle):
    return 1 / math.tan(math.radians(angle))

def update_visited_cells(cell):
    if cell[0] < len(visited_cells) and cell[1] < len(visited_cells[0]):
        visited_cells[cell[0]][cell[1]] = 'X'
    for row in visited_cells:
        print(''.join(row))
        

def triangulate_position(landmarks,angles):
    # Step 1: Compute modified beacon coordinates
    x1, y1 = beacons[0]['position']
    x2, y2 = beacons[1]['position']
    x3, y3 = beacons[2]['position']
    alpha1, alpha2, alpha3 = beacons[0]['angle'], beacons[1]['angle'], beacons[2]['angle']
    
    x1_prime, y1_prime = x1 - x2, y1 - y2
    x3_prime, y3_prime = x3 - x2, y3 - y2
    
    # Step 2: Compute cotangents
    T12 = cot(alpha2 - alpha1)
    T23 = cot(alpha3 - alpha2)
    T31 = (1 - T12 * T23) / (T12 + T23)
    
    # Step 3: Compute modified circle center coordinates
    x12_prime = x1_prime + T12 * y1_prime
    y12_prime = y1_prime - T12 * x1_prime
    x23_prime = x3_prime - T23 * y3_prime
    y23_prime = y3_prime + T23 * x3_prime
    x31_prime = (x3_prime + x1_prime) + T31 * (y3_prime - y1_prime)
    y31_prime = (y3_prime + y1_prime) - T31 * (x3_prime - x1_prime)
    
    # Step 4: Compute k'31
    k31_prime = x1_prime * x3_prime + y1_prime * y3_prime + T31 * (x1_prime * y3_prime - x3_prime * y1_prime)
    
    # Step 5: Compute D
    D = (x12_prime - x23_prime) * (y23_prime - y31_prime) - (y12_prime - y23_prime) * (x23_prime - x31_prime)
    
    if D == 0:
        raise ValueError("Cannot triangulate, D is zero")
    
    # Step 6: Compute the robot position
    xR = x2 + k31_prime * (y12_prime - y23_prime) / D
    yR = y2 + k31_prime * (x23_prime - x12_prime) / D
    
    return xR, yR


    estimated_position = triangulate_position(beacon_data)

    return triangulate_position(beacons, angles)
    
    # Initialize sum variables for x and y coordinates
    sum_x, sum_y = 0, 0
    
    # Total number of triangulations
    total_triangulations = 0
    
    # Perform triangulation for each combination of three beacons
    for i in range(len(beacon_positions)):
        # Select three beacons, skipping the i-th beacon
        selected_beacons = [beacon_positions[j] for j in range(len(beacon_positions)) if j != i]
        selected_angles = [angles[j] for j in range(len(angles)) if j != i]
        
        try:
            xR, yR = triangulate(selected_beacons, selected_angles)
            sum_x += xR
            sum_y += yR
            total_triangulations += 1
        except ValueError as e:
            # Handle errors for individual triangulations, if necessary
            print(e)
    
    # Calculate the average position
    if total_triangulations > 0:
        avg_x = sum_x / total_triangulations
        avg_y = sum_y / total_triangulations
        return (avg_x, avg_y)
    else:
        raise ValueError("Error computing the robot position: No successful triangulations.")

    try:
        robot_position = triangulate_position_with_four_beacons(beacon_positions, angles)
    except ValueError as e:
        print(e)


# Define a flag to control the rotation action
rotation_completed = False

def rotate(robot, degrees_to_rotate, rotation_completed):
    if rotation_completed:
        return True
    
    initial_heading = math.degrees(robot.get_compass_reading())  # Convert to degrees if necessary
    target_heading = (initial_heading + degrees_to_rotate) % 360
    
    robot.set_left_motors_velocity(5)
    robot.set_right_motors_velocity(-5)

    while True:
        current_heading = math.degrees(robot.get_compass_reading())  # Convert to degrees
        error = (target_heading - current_heading) % 360
        
        if error > 180:
            error -= 360
            
        if -0.75 <= error <= 0.75:
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            return True  # Rotation completed
        
    return False  # This should not be reached, but included for completeness



# Main control loop
while robot.experiment_supervisor.step(robot.timestep) != -1:
        
    print(f"Position: {position}")
    
    new_cell = determine_current_cell(x, y)
    if new_cell != current_cell:
        current_cell = new_cell
        update_visited_cells(current_cell)
    
   
    robot.set_left_motors_velocity(10)
    robot.set_right_motors_velocity(10)
    
    # Check if all cells have been visited to end the task
    if all('X' in row for row in visited_cells):
        print("All cells are visited!")
        break

    