
import numpy as np
import math

def create_local_goal(type, dist_lin, dist_theta):
    if type == 'circular':
        #use polar coordinates
        radius = dist_lin/dist_theta
        theta_loc = dist_theta
        print(f'The radius is: {radius}\n')
        print(f'Theta is: {theta_loc*180.0/math.pi:.3f}\n')
        x_loc = radius*np.cos(theta_loc) - radius    
        y_loc = radius*np.sin(theta_loc)
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
    print(f"====local goal======\n{goal_local}\n")

    return goal_local    

def create_goal(type, dist_lin, dist_theta, x, y, SO_2):
    if type == 'circular':
        #use polar coordinates
        radius = dist_lin/dist_theta
        theta_loc = dist_theta
        print(f'The radius is: {radius}\n')
        print(f'Theta is: {theta_loc*180.0/math.pi:.3f}\n')
        x_loc = radius*np.cos(theta_loc) - radius    
        y_loc = radius*np.sin(theta_loc)
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
    robot_pose = np.array([[SO_2[0,0], SO_2[0,1], x], 
                            [SO_2[1,0], SO_2[1,1], y], 
                            [0, 0, 1]]) 

    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    print(f"====local goal======\n{goal_local}\n")
    print(f"====robot_pose======\n{robot_pose}\n")
    
    goal_global = robot_pose@goal_local
    print(f"====global_goal======\n{goal_global}\n")

    return goal_global

def create_waypoints(type, total_linear, total_theta, num_points, x, y, theta):
    waypoints = []
    dist_lin = total_linear/num_points
    dist_theta = total_theta/num_points 
    #Create initial SO(2)
    SO_2 = np.array([[np.cos(theta), -np.sin(theta)], 
                    [np.sin(theta), np.cos(theta)]])
    SE_2 = np.array([[np.cos(theta), -np.sin(theta), x], 
                    [np.sin(theta), np.cos(theta), y],
                    [0.0, 0.0, 1.0]])
    goal_global = SE_2
    goal_local = create_local_goal(type, dist_lin, dist_theta)
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    print(f"====Initial Pose======\n{goal_global}\n")
    for i in range(num_points):
        #calculate global points (x, y, theta)
        #print(f"====ITERATION======\n{i}\n")
        #goal_global = create_goal(type, dist_lin, dist_theta, x, y, SO_2)
        goal_global = goal_global@goal_local
        waypoints.append((goal_global, dist_theta))
        # x = goal_global[0, 2]
        # y = goal_global[1, 2]
        # SO_2 = goal_global[0:2, 0:2]

    return waypoints

def main():
    #create_goal('angular', 5.0, 5.0, 0.0)
    radius = 2.0
    theta = math.pi/2
    waypoints = create_waypoints('angular', 0.0 , theta, 20, 5.0, 5.0, 0.0)
    print("The waypoints are:\n")
    for i in range(len(waypoints)):
        print(f'{waypoints[i][0]}\n')
        print(f'Angle increment: {waypoints[i][1]}\n')

if __name__=='__main__':
    main()