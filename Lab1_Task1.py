# Varennyam Joshi (U68760211)
# Lab 1 Task 1
# Path: /Users/varennyamjoshi/FAIRIS-Lite/WebotsSim/controllers/Lab1_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position (waypoint P0) listed in maze file
robot.move_to_start()

# defining waypoints p1 ---- p7
waypoints = [
    (-1, -1.5, math.pi),
    (-1.5, -1, math.pi / 2),
    (-1.5, 1, math.pi / 2),
    (-1, 1.5, 0),
    (0, 1.5, 0),
    (1.5, 0, 3 * math.pi / 2),
    (0, 0, math.pi / 2)
]

# Main Control Loop
for i in range(1, len(waypoints)):
    # Calculate distance and time between waypoints
    dx = waypoints[i][0] - waypoints[i - 1][0]
    dy = waypoints[i][1] - waypoints[i - 1][1]
    distance = math.sqrt(dx**2 + dy**2)
    time_to_travel = distance / robot.max_motor_velocity

    # Print information
    print(f"Waypoint {i}:")
    print("Distance:", distance)
    print("Time to travel:", time_to_travel)

#Initial velocities for right and left wheels 
vri = 0
vli = 0
#waypoint_time = robot.experiment_supervisor.getTime()

# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:

    print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Distance Sensor Values
    #print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
    #print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
    #print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
    #print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    #print("Lidar Front Reading", robot.get_lidar_range_image()[400])
    #print("Lidar Right Reading", robot.get_lidar_range_image()[600])
    #print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
    #print("Lidar Left Reading", robot.get_lidar_range_image()[200])
    
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    distance_front_right_wheel_traveled = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
    distance = (distance_front_left_wheel_traveled + distance_front_right_wheel_traveled)/2
    
    waypoint_time1 = (robot.experiment_supervisor.getTime())
    
    #Printing the required values
    print("Time", robot.experiment_supervisor.getTime())
    print("Left Wheel Velocity(rad/s):", vli)
    print("Right Wheel Velocity(rad/s):", vri)
    print("Distance(m):", distance)
    #print("Time Spent Between Waypoints(s):", waypoint_time1)
    print(" ")
        
    # Sets the robot's motor velocity to 20 rad/sec
    vri = 20
    vli = 20
    robot.set_right_motors_velocity(20)
    robot.set_left_motors_velocity(20)
    
    # Robot moves until the left wheel has covered 2.5m of distance in total
    if distance_front_left_wheel_traveled > 2.5:
        # Reaches P1
        vri = 10.5
        vli = 20
        robot.set_right_motors_velocity(10.5)
        robot.set_left_motors_velocity(20)
        
        # Robot moves until the left wheel has covered 3.3m of distance in  total
        if distance_front_left_wheel_traveled > 3.3:
            # Reaches P2
            vri = 20
            vli = 20
            robot.set_right_motors_velocity(20)
            robot.set_left_motors_velocity(20)
            
            # Robot moves until the left wheel has covered 5.295m of distance in  total
            if distance_front_left_wheel_traveled > 5.295:
                # Reaches P3
                vri = 10.75
                vli = 20
                robot.set_right_motors_velocity(10.75)
                robot.set_left_motors_velocity(20)
                
                # Robot moves until the left wheel has covered 6.09m of distance in  total
                if distance_front_left_wheel_traveled > 6.09:
                    # Reaches P4
                    vri = 20
                    vli = 20
                    robot.set_right_motors_velocity(20)
                    robot.set_left_motors_velocity(20)
                    
                    # Robot moves until the left wheel has covered 7.12m of distance in  total
                    if distance_front_left_wheel_traveled > 7.12:
                        # Reaches P5
                        vri = 17.157
                        vli = 20
                        robot.set_right_motors_velocity(17.157)
                        robot.set_left_motors_velocity(20)
                        #9.455, 14.155, 7.09
                        
                        # Robot moves until the left wheel has covered 9.7m of distance in  total
                        if distance_front_left_wheel_traveled > 9.7:
                            # Reaches P6
                            vri = 14.593
                            vli = 20
                            robot.set_right_motors_velocity(14.593)
                            robot.set_left_motors_velocity(20)
                            
                            # Robot moves until the left wheel has covered 12.65m of distance in  total
                            if distance_front_left_wheel_traveled > 12.65:
                                # Reaches P7
                                # Then the robot stops and prints the final readings
                                vri = 0
                                vli = 0
                                robot.stop()
                                print("Simulation Time", robot.experiment_supervisor.getTime())
                                print("Left Wheel Velocity(rad/s):", vli)
                                print("Right Wheel Velocity(rad/s):", vri)
                                print("Distance(m):", distance)
                                print(" ")
                                # The loop is breaked after robot reaches the final waypoint 
                                break
                                




