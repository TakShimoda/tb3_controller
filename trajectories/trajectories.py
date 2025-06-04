# A collection of trajectories of multi-robots in the 6x4 grid

# Dictionary to map the names (keys) to integers for easy calling
file_keys = {
    0: '2_robot_lawnmower_4x4',
    1: '2_robot_zigzag',
    2: '2_robot_lawnmower_5x4'
}

# Dictionary containing the different multi-robot trajectory scenarios
    # Keys: name of trajectory
    # Value: list of tuples: (csv file, starting pose, name on plot)
files = {
    '2_robot_lawnmower_4x4': [
        ("lawnmower_4x4.csv", (0, 0, 0), "lawnmower"),
        ("lawnmower_4x4.csv", (4, 4, 180), "lawnmower2")],
    '2_robot_zigzag': [
        ("zigzag.csv", (0, 0, 0), "zigzag1"),
        ("zigzag.csv", (4, 4, 180), "zigzag2")],
    '2_robot_lawnmower_5x4': [
        ("lawnmower_5x4.csv", (0, 0, 0), "robot1"),
        ("lawnmower_4x5.csv", (0, 4, 0), "robot2")
    ]
}