# A collection of trajectories of multi-robots in the 6x4 grid

# Dictionary to map the names (keys) to integers for easy calling
file_keys = {
    0: '2_robot_lawnmower_4x4',
    1: '2_robot_zigzag',
    2: '2_robot_lawnmower_5x4',
    3: '3_robot_t1'
}

# Dictionary containing the different multi-robot trajectory scenarios
    # Keys: name of trajectory
    # Value: list of tuples: (csv file, starting pose, name on plot)
files = {
    '2_robot_lawnmower_4x4': [
        ("lawnmower_4x4.csv", (0, 0, 90), "robot1"),
        ("lawnmower_4x4_2.csv", (0.5, 4, 00), "robot2")
    ],
    '2_robot_zigzag': [
        ("zigzag.csv", (0, 0, 0), "zigzag1"),
        ("zigzag.csv", (4, 4, 180), "zigzag2")
    ],
    '2_robot_lawnmower_5x4': [
        ("lawnmower_5x4.csv", (0, 0, 0), "robot1"),
        ("lawnmower_4x5.csv", (1, 4, 0), "robot2")
    ],
    '3_robot_t1': [
        ("t1_r1.csv", (0, 0, 0), "r1"),
        ("t1_r2.csv", (-1, -1, 0), "r2"),
        ("t1_r3.csv", (1, 1, 0), "r3")
    ]
}