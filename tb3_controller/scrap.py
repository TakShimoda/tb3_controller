#File for some scrap functions. Could be useful later


'''
Maybe useful later
    - It uses repeated calculation of local goals which is redundant for equidistant waypoints
    - However, it may be useful if more complex waypoints are needed in the future
Create goal: create goal in global coordinates
    Inputs: 
    - type: (circ/linear/angular)
    - current pose: (x, y s0_2). so_2 is used so we don't have to convert to get angles
    Outputs: goal in global coordinates
'''
def create_goal(self, type, dist_lin, dist_theta, x, y, SO_2):
    if type == 'circular':
    #use polar coordinates
        radius = dist_lin/dist_theta
        theta_loc = dist_theta
        x_loc = radius*np.cos(theta_loc) - radius
        y_loc = radius*np.sin(theta_loc)
    elif type == 'linear':
        x_loc = 0.0
        y_loc = dist_lin
        theta_loc = 0.0
    else: #turning/angular
        x_loc = 0.0
        y_loc = 0.0
        theta_loc = dist_theta

    #Define goal in local coordinates
    goal_local = np.array([[np.cos(theta_loc), -np.sin(theta_loc), x_loc], 
                            [np.sin(theta_loc), np.cos(theta_loc), y_loc], 
                            [0, 0, 1]])
    robot_pose = np.array([[SO_2[0,0], SO_2[0,1], x], 
                            [SO_2[1,0], SO_2[1,1], y], 
                            [0, 0, 1]])  
    #Define goal in global coordinates
    goal_global = robot_pose@goal_local

    return goal_global

'''
Basic P: Basic P example for linear motion (not for actual turtlebot)
    Inputs: feedback, goal_x, goal_y, goal_theta, goal_handle
    Outputs: None
''' 
# def basic_P(self, feedback, diff_x, diff_y, diff_theta, goal_x, goal_y, goal_theta, goal_handle):
#     while not (abs(diff_x) < self.diff_x and abs(diff_y) < self.diff_y and abs(diff_theta) < self.diff_theta):

#         #Update poses by adding P*difference, only if difference is still above threshold
#         self.pose_x += self.P*diff_x*(abs(diff_x) > self.diff_x)
#         self.pose_y += self.P*diff_y*(abs(diff_y) > self.diff_y)
#         self.pose_theta += self.P*diff_theta*(abs(diff_theta) > self.diff_theta)

#         #Update pose differential
#         diff_x = goal_x - self.pose_x
#         diff_y = goal_y - self.pose_y
#         diff_theta = goal_theta - self.pose_theta

#         #Send feedback
#         feedback._x_delta = diff_x
#         feedback._y_delta = diff_y
#         feedback._theta_delta = diff_theta
        
#         goal_handle.publish_feedback(feedback)
#         self.get_logger().info(
#             f'Current pose {self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta:.3f}')
#         time.sleep(self.delay)
#     return

'''
Get pose callback: callback for getting pose DON'T NEED BECAUSE WE BLOCK OUR CODE TO GET POSE
    Inputs: future 
    Outputs: None
'''
# def get_pose_callback(self, future):
#     try:
#         response = future.result()
#         self.get_logger().info(
#             f"Got pose: {response.x:.3f}, {response.y:.3f}, {response.theta:.3f} ")
#     except Exception as e:
#         self.get_logger().error("Service call failed %r" %(e,))