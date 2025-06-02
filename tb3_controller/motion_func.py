'''
    Functions for motion (e.g. creating waypoints, local goals)
'''

import rclpy
import numpy as np
import math

'''
Create local goal: create local goal once, to repeatedly right-multiply into initial robot-pose
    Inputs: 
    - type: (circ/linear/angular)
    - dist_lin: total linear distance (m)
    - dist_theta: total angular distance (rad)
    Outputs: goal in local coordinates coordinates
'''
def create_local_goal(type, dist_lin, dist_theta):
    if type == 'circular':
        #use polar coordinates
        radius = dist_lin/abs(dist_theta)
        theta_loc = dist_theta
        sign = dist_theta/abs(dist_theta)
        print(f'The radius is: {radius}\n')
        print(f'Theta is: {theta_loc*180.0/math.pi:.3f}\n')
        x_loc = (radius*np.cos(theta_loc) - radius)*sign    
        y_loc = radius*np.sin(theta_loc)*sign
    elif type == 'linear':
        x_loc = 0.0
        y_loc = dist_lin
        theta_loc = 0.0
    else: #angular
        x_loc = 0.0
        y_loc = 0.0
        theta_loc = dist_theta

    goal_local = np.array([[np.cos(theta_loc), -np.sin(theta_loc), x_loc], 
                            [np.sin(theta_loc), np.cos(theta_loc), y_loc], 
                            [0, 0, 1]])

    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    return goal_local   

'''
Create waypoints: create equidistant waypoints in global coordinates, each as goals for server to complete
    - Moreover, this would take a larger goal (e.g. quarter circle) and break it down to smaller goals
    - Also, the global goal returned on one iteration can serve as the pose in the next iteration 
        to easily make consecutive equidistant waypoints
    - Also returns angle increments as it's hard to calculate RPY off of SO(2)
    Inputs: 
        - type: circular, linear, or other(i.e. angular)
        - total_linear: total linear distance
        - total_theta: total angular distance
        - num_points: number of points
        - (x, y, theta): current robot pose in Cartesian global coordinates 
    Outputs: waypoints [(SE(2) as np.array.flatten(), linear increment, angle increment, global theta)]
'''
def create_waypoints(type, total_linear, total_theta, num_points, x, y, theta):
    waypoints = []
    dist_lin = total_linear/num_points
    dist_theta = total_theta/num_points 
    #Create initial SE(2)
    SE_2 = np.array([[np.cos(theta), -np.sin(theta), x], 
                [np.sin(theta), np.cos(theta), y],
                [0.0, 0.0, 1.0]], dtype=np.float32)
    goal_global = SE_2

    #Create local goal to repeatedly right-multiply to robot pose to accumulate waypoints
    goal_local = create_local_goal(type, dist_lin, dist_theta)
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    print(f"====Initial Robot Pose======\n{goal_global}\n")
    print(f'====local waypoint goal for type {type}, repeated {num_points} times======\n{goal_local}\n')

    for i in range(num_points):
        #calculate global points in Cartesian
        goal_global = (goal_global@goal_local).astype(dtype=np.float32)
        #add the increment to the global theta
        theta += dist_theta
        theta -= (2*math.pi*(theta>math.pi))
        waypoints.append((goal_global.flatten(), dist_lin, dist_theta, theta))

    return waypoints