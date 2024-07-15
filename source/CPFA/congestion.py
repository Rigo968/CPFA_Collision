import pandas as pd
import numpy as np

def run_congestion_logic(all_trajectories):

    # # all_trajectories = {'robot_0': [x,y], 'robot_1': [x,y], ...}

    trajectories = []
    congested_robots = []

    for i, array in enumerate(all_trajectories):
        coords = []

        coords.append(all_trajectories[i][0])
        coords.append(all_trajectories[i][1])
        trajectories.append(coords)
    #print(all_trajectories)
    # turn trajectories into a numpy array with shape (16, 1, 2)
    trajectories = np.array(trajectories)
    trajectories = trajectories.reshape((16, 1, 2))

    # # all trajectories should have be a list of the 16 trajectories in string format
    # # this will convert them to a floats and numpy array with shape (16, 2, 1)
    # for i, array in enumerate(all_trajectories):
    #     x_coords = []
    #     y_coords = []

    #     for points in array:
    #         for point in points:
    #             try:
    #                 x, y = map(float, point.split(","))
    #                 x_coords.append(x)
    #                 y_coords.append(y)
    #             except ValueError:
    #                 continue
    #     new_array = np.array([x_coords, y_coords])
    #     trajectories.append(new_array)

    # trajectories = np.array(trajectories)
    # trajectories = np.transpose(trajectories, (0, 2, 1))

    # # Calculate the average distance between each robot and all other robots at each timestep in their trajectories. 
    # # I can later define what certain distance will give the robot a point
    def euclidean_distance(x1, y1, x2, y2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def count_robots_in_radius(trajectories, radius, radius_min = 0):

        num_robots = trajectories.shape[0]
        num_timesteps = trajectories.shape[1]
        num_robots_in_radius = np.zeros((num_robots, num_timesteps), dtype=int)

        for t in range(num_timesteps):
            for i in range(num_robots):
                for j in range(num_robots):
                    if i != j and euclidean_distance(trajectories[i, t, 0], trajectories[i, t, 1],
                                                    trajectories[j, t, 0], trajectories[j, t, 1]) <= radius and euclidean_distance(trajectories[i, t, 0], trajectories[i, t, 1],
                                                    trajectories[j, t, 0], trajectories[j, t, 1]) >= radius_min:
                        num_robots_in_radius[i, t] += 1

        return num_robots_in_radius

    # # Use robot's proximity with weights

    radius = 1
    radius_min = 0.5

    num_robots_in_radius = count_robots_in_radius(trajectories, radius, radius_min)
    num_timesteps = num_robots_in_radius.shape[1]
    num_robots = num_robots_in_radius.shape[0]
    robot_density_high_radius = num_robots_in_radius.T.reshape(num_timesteps, num_robots)

    radius = 0.5
    radius_min = 0.25

    num_robots_in_radius = count_robots_in_radius(trajectories, radius, radius_min)
    num_timesteps = num_robots_in_radius.shape[1]
    num_robots = num_robots_in_radius.shape[0]

    robot_density_medium_radius = num_robots_in_radius.T.reshape(num_timesteps, num_robots)
    robot_density_medium_radius *= 2

    radius = 0.25

    num_robots_in_radius = count_robots_in_radius(trajectories, radius)
    num_timesteps = num_robots_in_radius.shape[1]
    num_robots= num_robots_in_radius.shape[0]

    robot_density_low_radius = num_robots_in_radius.T.reshape(num_timesteps, num_robots)
    robot_density_low_radius *= 3

    # # Calculate the average distance between each robot and 
    # # all other robots at each timestep in their trajectories. 
    # # I can later define what certain distance will give the robot a point
    # num_robots = 16
    # distances_from_robots = [[] for _ in range(num_robots)]

    # for r in range(num_robots):
    #     for i in range(len(trajectories[0])):
    #         distances = []
    #         for j in range(num_robots):
    #             if j != r:
    #                 distance = euclidean_distance(trajectories[r][i, 0], trajectories[r][i, 1], trajectories[j][i, 0], trajectories[j][i, 1])
    #                 distances.append(distance)
    #         avg_distance = np.mean(distances)
    #         distances_from_robots[r].append(avg_distance)

    # distances_from_robots = np.array(distances_from_robots)
    # distances_from_robots = distances_from_robots.T

    # median_distances_from_robots = 2.127028932039728

    # distances_more_than_avg = np.where(distances_from_robots < median_distances_from_robots, 1, 0)

    # # If robot stays at the same position, it will receive one point
    # # def track_staying_points(trajectories):
    # #     num_robots = trajectories.shape[0]
    # #     num_timesteps = trajectories.shape[1]
    # #     staying_indices = np.zeros((num_robots, num_timesteps), dtype=int)

    # #     for i in range(num_robots):
    # #         for t in range(1, num_timesteps):
    # #             if (trajectories[i, t, 0] == trajectories[i, t-1, 0] and
    # #                 trajectories[i, t, 1] == trajectories[i, t-1, 1]):
    # #                 staying_indices[i, t] = 1

    # #     return staying_indices

    # # staying_indices = track_staying_points(trajectories)

    # # staying_indices = staying_indices.T.reshape(num_timesteps, num_robots)

    total_points = robot_density_low_radius + robot_density_medium_radius + robot_density_high_radius

    # # loop through total points
    # for i in range(total_points.shape[1]):
    #     if total_points[1, i] > 10:
    #         congested_robots.append(i)
    #         # perform re routing logic here
    for i, value in np.ndenumerate(total_points):
        if value > 3:
            congested_robots.append(i[1])  # i[1] is the column index            

    #print(total_points)
    return congested_robots