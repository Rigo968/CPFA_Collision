import matplotlib.pyplot as plt
import math
import pandas as pd
import re
def parse_trajectory_file(file_path, interval):
    # Define a regex pattern to capture robot, trajectory, and (x, y) positions
    pattern = r"Robot:\s*(\w+),\s*Trajectory:\s*(\d+),\s*Positions:\s*([\d.-]+),([\d.-]+)"
    
    # List to hold data for the DataFrame
    data = []

    # Variables to track the current robot and trajectory
    current_robot = None
    current_trajectory = None

    # Open and read the file
    try:
        with open(file_path, 'r') as file:
            for line in file:
                # Find all matches in each line
                matches = re.findall(pattern, line)

                # Process each match
                for robot, traj, x, y in matches:
                    robot_id = robot.strip()
                    trajectory_id = int(traj.strip())
                    x = float(x)
                    y = float(y)

                    # Check if we have a new robot/trajectory combination
                    if (robot_id != current_robot) or (trajectory_id != current_trajectory):
                        # Reset for new combination
                        current_robot = robot_id
                        current_trajectory = trajectory_id
                    
                    # Append the data for this match
                    data.append({
                        'Robot': robot_id,
                        'Trajectory': trajectory_id,
                        'X': x,
                        'Y': y
                    })

    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return pd.DataFrame()  # Return an empty DataFrame on error

    # Create a DataFrame from the data list
    df = pd.DataFrame(data)

    # Debug: Check if DataFrame is empty
    if df.empty:
        print("No data parsed. DataFrame is empty.")
        return df
    try:
        df.sort_values(by=['Robot', 'Trajectory'], inplace=True)
    except KeyError as e:
        print(f"KeyError during sorting: {e}")
        return df  # or handle as needed

    # Assuming add_timesteps is defined elsewhere
    df = add_timesteps(df)

    # Filter for valid timesteps
    valid_timesteps = [i for i in frange(0, df['Timestep'].max() + interval, interval)]
    df = df[df['Timestep'].isin(valid_timesteps)]

    return df

def frange(start, stop, step):
    while start < stop:
        yield round(start, 10)  # Round to avoid floating-point issues
        start += step

def add_timesteps(df):
    # Constant timestep increment
    timestep_increment = 0.03125
    
    # Group by 'robot' and 'trajectory'
    grouped = df.groupby(['Robot', 'Trajectory'])

    # Calculate the timestep for each group
    df['Timestep'] = grouped.cumcount() * timestep_increment

    return df

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

# Save DataFrame to a CSV file
#output_file_path = "parsed_trajectory_data.csv"
#df_with_timesteps.to_csv(output_file_path, index=False)

def Distance_Storage_For_Bot(Robot, interval):
    df = parse_trajectory_file(file_path, 0.03125)
    df_interval = parse_trajectory_file(file_path, interval)
    f_sec = df_interval[df_interval['Robot'] == Robot]
    f_step = df[df['Robot'] == Robot]
  # Change to right=False for exclusive upper bound
    total_dist = []
    short_dist = []
    # Calculate total distances within each time bin
    for traj in f_step['Trajectory'].unique():  # Loop through all trajectories
        f_step_traj = f_step[f_step['Trajectory'] == traj]  # Filter by trajectory
        timestep = f_step_traj['Timestep'].max()
        if timestep % interval == 0:
            max_timestep = int(timestep)
        else:
            max_timestep = int(timestep) - (int(timestep) % interval)
        for start_time in range(0, int(max_timestep) + 1, interval):
            if start_time + interval <= max_timestep:  # Stop at the last 5-second interval
                current_range = f_step_traj[(f_step_traj['Timestep'] >= start_time) & (f_step_traj['Timestep'] <= start_time + interval)]
            else:
              break
            if current_range.shape[0] < (32*interval) + 1:
              continue
            x_values = current_range['X'].to_numpy()
            y_values = current_range['Y'].to_numpy()
            distance = calculate_distance(x_values, y_values)
            total_dist.append(distance)
    # Calculate short distances
    for traj in f_sec['Trajectory'].unique():  # Loop through all trajectories
        f_traj = f_sec[f_sec['Trajectory'] == traj]
        for i in range(len(f_traj) - 1):  # Ensure you are not going out of bounds
            x1, y1 = f_traj['X'].iloc[i], f_traj['Y'].iloc[i]
            x2, y2 = f_traj['X'].iloc[i + 1], f_traj['Y'].iloc[i + 1]
            distance = calculate_distance_two_points(x1, x2, y1, y2)
            short_dist.append(distance)
    return short_dist, total_dist

def congestion_detection(Robot, interval, threshold):
    short, actual = Distance_Storage_For_Bot(Robot, interval)
    congestion = []
    for i in range(len(actual)):
        difference = actual[i] - short[i]
        if difference > threshold:
            congestion.append(i)
    return congestion

def check_congestion(timestep, intervals):
    for start, end in intervals:
        if start <= timestep <= end:
            return 1
    return 0

def get_congestion_coordinates(df, congestion, second_interval, Robot):
    timesteps = []
    for i in range(len(congestion)):
        timestep_start = congestion[i] * second_interval
        timestep_end = timestep_start + second_interval
        timesteps.append([timestep_start, timestep_end])
    Congestion = df[df['Robot'] == Robot].copy()
    Congestion['Congestion'] = Congestion['Timestep'].apply(lambda x: check_congestion(x, timesteps))
    return Congestion

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

def UpdateDf(df, intervals, threshold):
    df_updated = df.copy()
    df_updated['Congestion'] = pd.NA
    for i in df['Robot'].unique():
        df_new = get_congestion_coordinates(df, congestion_detection(i, intervals, threshold), intervals, i)
        #print(df_new)

        df_updated = pd.merge(df_updated, df_new[['Robot', 'Trajectory', 'Timestep', 'X', 'Y', 'Congestion']], on=['Robot', 'Trajectory', 'Timestep', 'X', 'Y'], how='left')
        #print(df_updated[df_updated['Robot'] == 'F01'])

        df_updated['Congestion'] = df_updated['Congestion_x'].combine_first(df_updated['Congestion_y'])
        df_updated.drop(columns=['Congestion_x', 'Congestion_y'], inplace=True)
    return df_updated


file_path = './results/cluster_CPFA_r16_tag128_8by8_quard_arena_0_iAntDroppedTrajData1.txt'

increment = 3
th = 0.005
df = parse_trajectory_file(file_path, 0.03125)
df_interval = parse_trajectory_file(file_path, increment)

#print(df)
#print(df_interval)


Congestion_df = UpdateDf(df, increment, th)
Congestion_df = Congestion_df[Congestion_df['Congestion'] == 1]

#plot_robot_trajectory_with_collisions(Congestion_df)

#use get_collision_coordinates function if you want a specific bot being plotted
#Congestion_bot = get_congestion_coordinates(df, congestion_detection("F32", increment, th), increment, "F32")
#plot_robot_trajectory_with_collisions(Congestion_bot)

#print(Congestion_df)

output_file_path = "parsed_trajectory_data.csv"

Congestion_df.to_csv(output_file_path, index=False, mode='w')