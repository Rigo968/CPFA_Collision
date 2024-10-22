import matplotlib.pyplot as plt
import math
import pandas as pd

def parse_trajectory_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    trajectories = {}
    current_robot = None
    current_trajectory = None
    nest_counter = None
    collision_counter = None
    timestep = None

    for line in lines:
        line = line.strip()
        if not line:
            continue

        # Detect the robot, nest counter, collision counter, and trajectory headers
        if line.startswith("Robot:"):
            parts = line.split(',')
            current_robot = parts[0].split(": ")[1].strip()
            nest_counter = int(parts[1].split(": ")[1].strip())
            collision_counter = int(parts[2].split(": ")[1].strip())
            current_trajectory = int(parts[3].split()[-1][:-1])

            # Initialize the robot in the dictionary if it's new
            if current_robot not in trajectories:
                trajectories[current_robot] = {}

            # Initialize the trajectory if it's new and store nest/collision counters
            if current_trajectory not in trajectories[current_robot]:
                trajectories[current_robot][current_trajectory] = {
                    'nest_counter': nest_counter,
                    'collision_counter': collision_counter,
                    'timesteps': {}
                }

        # Extract (x, y) points from each line
        elif ";" in line:
            pts = line.split(';')
            for pt in pts:
                p = pt.split(",")
                if len(p) >= 3:
                    timestep = float(p[0].split()[2])
                    x = float(p[1].strip())
                    y = float(p[2].strip().split()[0])
                    trajectories[current_robot][current_trajectory]['timesteps'][timestep] = {
                                'points': []
                            }
                    trajectories[current_robot][current_trajectory]['timesteps'][timestep]['points'].append((x, y))
    return trajectories

def calculate_distance(x_vals, y_vals):
    """
    Calculate the total distance traveled for a given set of points.
    """
    total_distance = 0
    for i in range(1, len(x_vals)):
        distance = math.sqrt((x_vals[i] - x_vals[i - 1])**2 + (y_vals[i] - y_vals[i - 1])**2)
        total_distance += distance
    return total_distance

def calculate_distance_two_points(x1, x2, y1, y2):
    """
    Calculate the distance between two points (x1, y1) and (x2, y2).
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def generate_trajectory_table(trajectories, second_intervals):
    """
    Generate a table with Total Distance, Nest Counter, Collision Counter, and Congested columns
    and display it using pandas DataFrame.
    Trajectories with zero total distance will not be added to the table.
    """
    table_data = []
    # Iterate through the robots and their trajectories
    for robot, robot_trajectories in trajectories.items():
        for trajectory_num, data in robot_trajectories.items():
            x_vals = []
            y_vals = []
            all_timesteps = []
            for timestep, timestep_data in data['timesteps'].items():
                points = timestep_data['points']
                x_vals.extend([point[0] for point in points])
                y_vals.extend([point[1] for point in points])
                all_timesteps.extend([(timestep, point[0], point[1]) for point in points])

            if not x_vals:
                continue
            # Append the relevant data to the table
            #Each Step is 0.03125 seconds
            for i, (timestep, x, y) in enumerate(all_timesteps):
              if second_intervals > 0 and i % (32 * second_intervals) == 0:
                  table_data.append([robot, trajectory_num, timestep, x, y])
              elif second_intervals == 0:
                  table_data.append([robot, trajectory_num, timestep, x, y])

    # Convert table_data into a DataFrame
    result_df = pd.DataFrame(table_data, columns=['Robot', 'Trajectory', 'Timestep(Sec.)', 'X', 'Y'])
    return result_df

def Distance_Storage_For_Bot(per_step, Robot, interval, trajectories):
    per_sec = generate_trajectory_table(trajectories, interval)
    f = per_sec[per_sec['Robot'] == Robot]
    f_1 = per_step[per_step['Robot'] == Robot].copy()
  # Change to right=False for exclusive upper bound
    total_dist = []
    short_dist = []
    # Calculate total distances within each time bin
    for traj in f_1['Trajectory'].unique():  # Loop through all trajectories
        f_1_traj = f_1[f_1['Trajectory'] == traj]  # Filter by trajectory
        timestep = f_1_traj['Timestep(Sec.)'].max()
        if timestep % interval == 0:
            max_timestep = int(timestep)
        else:
            max_timestep = int(timestep) - (int(timestep) % interval)
        for start_time in range(0, int(max_timestep) + 1, interval):
            if start_time + interval <= max_timestep:  # Stop at the last 5-second interval
                current_range = f_1_traj[(f_1_traj['Timestep(Sec.)'] >= start_time) & (f_1_traj['Timestep(Sec.)'] <= start_time + interval)]
            else:
              break
            if current_range.shape[0] < (32*interval) + 1:
              continue
            x_values = current_range['X'].to_numpy()
            y_values = current_range['Y'].to_numpy()
            distance = calculate_distance(x_values, y_values)
            total_dist.append(distance)
    # Calculate short distances
    for traj in f['Trajectory'].unique():  # Loop through all trajectories
        f_traj = f[f['Trajectory'] == traj]
        for i in range(len(f_traj) - 1):  # Ensure you are not going out of bounds
            x1, y1 = f_traj['X'].iloc[i], f_traj['Y'].iloc[i]
            x2, y2 = f_traj['X'].iloc[i + 1], f_traj['Y'].iloc[i + 1]
            distance = calculate_distance_two_points(x1, x2, y1, y2)
            short_dist.append(distance)
    return short_dist, total_dist

def collision_detection(Robot, interval, threshold, df, traj):
    short, actual = Distance_Storage_For_Bot(df, Robot, interval, traj)
    collision = []
    for i in range(len(actual)):
        difference = actual[i] - short[i]
        if difference > threshold:
            collision.append(i)
    return collision

def check_collisions(timestep, intervals):
    for start, end in intervals:
        if start <= timestep <= end:
            return 1
    return 0

def get_collision_coordinates(per_step, collision, second_interval, Robot):
    timesteps = []
    for i in range(len(collision)):
        timestep_start = collision[i] * second_interval
        timestep_end = timestep_start + second_interval
        timesteps.append([timestep_start, timestep_end])
    Collision = per_step[per_step['Robot'] == Robot].copy()
    Collision['Congestion'] = Collision['Timestep(Sec.)'].apply(lambda x: check_collisions(x, timesteps))
    return Collision

def plot_robot_trajectory_with_collisions(df):
    colors = plt.cm.get_cmap('tab10', len(df['Robot'].unique()))
    plt.figure(figsize=(10, 10))
    collision_points = df[df['Congestion'] == 1]

    for i, robot in enumerate(df['Robot'].unique()):
      robot_data = df[df['Robot'] == robot]
      for trajectory in robot_data['Trajectory'].unique():
        trajectory_data = robot_data[robot_data['Trajectory'] == trajectory]
        plt.plot(trajectory_data['X'], trajectory_data['Y'],
                 label=f'Robot {robot}, Trajectory {trajectory}',
                 color=colors(i),
                 alpha=1,
                 linewidth = 3)
    # Plot the overall trajectory
    plt.scatter(collision_points['X'], collision_points['Y'], color='yellow', label='Collision Points', alpha = 0.3, s = 10, zorder = 2)

    plt.title('Robot Trajectory with Collision Highlighting')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.grid()
    plt.show()

def UpdateDf(df, intervals, threshold, traj):
    df_updated = df.copy()
    df_updated['Congestion'] = pd.NA
    for i in df['Robot'].unique():
        df_new = get_collision_coordinates(df, collision_detection(i, intervals, threshold, df, traj), intervals, i)
        #print(df_new)

        df_updated = pd.merge(df_updated, df_new[['Robot', 'Trajectory', 'Timestep(Sec.)', 'X', 'Y', 'Congestion']], on=['Robot', 'Trajectory', 'Timestep(Sec.)', 'X', 'Y'], how='left')
        #print(df_updated[df_updated['Robot'] == 'F01'])

        df_updated['Congestion'] = df_updated['Congestion_x'].combine_first(df_updated['Congestion_y'])
        df_updated.drop(columns=['Congestion_x', 'Congestion_y'], inplace=True)
    return df_updated

file_path = '../../results/cluster_CPFA_r16_tag128_8by8_quard_arena_0_iAntDroppedTrajData1.txt'
traj = parse_trajectory_file(file_path)
interval = 4
th = 0.01


df_perstep = generate_trajectory_table(traj, 0)
df_perinterval = generate_trajectory_table(traj, interval)
Collision_df = UpdateDf(df_perstep, interval, th, traj)
plot_robot_trajectory_with_collisions(Collision_df)

#use get_collision_coordinates function if you want a specific bot being plotted
collision_bot = get_collision_coordinates(df_perstep, collision_detection("F32", interval, th, df_perstep, traj), interval, "F32")
plot_robot_trajectory_with_collisions(collision_bot)

print(Collision_df)
print(df_perinterval)
print(df_perstep)