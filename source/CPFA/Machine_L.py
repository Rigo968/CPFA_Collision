import matplotlib.pyplot as plt
import math
import pandas as pd
import re
import numpy as np
import pickle
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report



def parse_trajectory_file(file_path, interval):
    # Define a regex pattern to capture robot, trajectory, and (x, y) positions
    pattern = r"Robot:\s*(\w+),\s*Trajectory:\s*(\d+),\s*Positions:\s*([\d.-]+),([\d.-]+), \s*Game:\s*(\d+)"
    
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
                for robot, traj, x, y, game in matches:
                    robot_id = robot.strip()
                    trajectory_id = int(traj.strip())
                    x = float(x)
                    y = float(y)
                    game_count = int(game)


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
                        'Y': y,
                        'Game': game_count
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

def plot_robot_trajectory_with_collisions(df):
    colors = plt.cm.get_cmap('tab10', len(df['Robot'].unique()))
    plt.figure(figsize=(15, 15))
    for game in df['Game'].unique():
        game_data = df[df['Game'] == game]
        for i, robot in enumerate(game_data['Robot'].unique()):
            robot_data = game_data[game_data['Robot'] == robot]
            for trajectory in robot_data['Trajectory'].unique():
                trajectory_data = robot_data[robot_data['Trajectory'] == trajectory]
                collision_points = trajectory_data[trajectory_data['Congestion'] == 1]
                plt.plot(trajectory_data['X'], trajectory_data['Y'],
                         label=f'Robot {robot}, Trajectory {trajectory}',
                         color=colors(i),
                         alpha=1,
                         linewidth=3)
                
                # Plot the collision points (highlighted in red)
                plt.plot(collision_points['X'], collision_points['Y'],
                         color='red', alpha=1, linewidth=3, zorder=100)

    plt.title('Robot Trajectory with Machine Learning')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.grid()
    plt.show()


file_path = './results/cluster_CPFA_r16_tag128_8by8_quard_arena_0_iAntDroppedTrajData1.txt'

df = parse_trajectory_file(file_path, 0.03125)
df = df[df['Timestep'] >= 2.000]

# Compute the velocity for each group of 'Robot' and 'Trajectory'
velocity = df.groupby(['Robot', 'Trajectory']).apply(
    lambda group: ((group['X'].diff() ** 2 + group['Y'].diff() ** 2) ** 0.5) / 0.03125
)

# Convert the result from a multi-index series to a single-column series
velocity = velocity.reset_index(level=['Robot', 'Trajectory'], drop=True)

# Assign the 'Velocity' column in the DataFrame
df['Velocity'] = velocity

# Fill any NaN values in the 'Velocity' column with 0
df['Velocity'].fillna(0, inplace=True)
with open('./source/CPFA/congestion_model.pkl', 'rb') as file:
    model = pickle.load(file)

X_new = df[['Velocity', 'X', 'Y']]
predictions = model.predict(X_new)  # Replace X_new with your new data
df['Congestion'] = predictions

Congestion_df = df[df['Congestion'] == 1]
Congestion_df = Congestion_df.groupby(['Robot', 'Trajectory']).first().reset_index()
output_file_path = "./source/CPFA/parsed_trajectory_data.csv"
Congestion_df.to_csv(output_file_path, index=False, mode='w')

