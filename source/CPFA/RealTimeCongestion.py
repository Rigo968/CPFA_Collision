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
def Distance_Storage_For_Bot(Robot, interval, slide_intervals = 32):
    df = parse_trajectory_file(file_path, 0.03125)
    f_step = df[df['Robot'] == Robot]
    short_dist = []
    total_dist = []  # To store distances for each window

    for game in f_step['Game'].unique():
        f_step_game = f_step[f_step['Game'] == game]

        for traj in f_step_game['Trajectory'].unique():
            f_step_traj = f_step_game[f_step_game['Trajectory'] == traj]  # Filter data for the current trajectory
            
            # Loop through the filtered trajectory data using a sliding window
            for i in range(0, len(f_step_traj) - interval + 1, slide_intervals):
                    # Get the points in the current window
                window = f_step_traj.iloc[i:i + interval]

                x_values = window['X'].to_numpy()
                y_values = window['Y'].to_numpy()
                distance = calculate_distance(x_values, y_values)
                total_dist.append(distance)
                short_distance = calculate_distance_two_points(x_values[0], x_values[-1], y_values[0], y_values[-1])
                short_dist.append(short_distance)

    return short_dist, total_dist

def congestion_detection(Robot, interval):
    short, actual = Distance_Storage_For_Bot(Robot, interval)
    congestion = []
    diff = []
    for i in range(len(actual)):
        difference = actual[i] - short[i]
        diff.append(difference)
    mean_diff = np.mean(diff)
    std_diff = np.std(diff)
    threshold = mean_diff + 2 * std_diff
    congestion = [i for i, differ in enumerate(diff) if differ > threshold]

    return congestion

def check_congestion(timestep, intervals):
    for start, end in intervals:
        if start <= timestep <= end:
            return 1
    return 0

def get_congestion_coordinates(df, congestion, interval, Robot):
    timesteps = []
    for i in range(len(congestion)):
        timestep_start = congestion[i] 
        timestep_end = timestep_start + interval
        timesteps.append([timestep_start, timestep_end])
        
    Congestion = df[df['Robot'] == Robot].copy()
    Congestion['Congestion'] = Congestion['Timestep'].apply(lambda x: check_congestion(x, timesteps))
    return Congestion

def plot_robot_trajectory_with_collisions(df):
    colors = plt.cm.get_cmap('tab10', len(df['Robot'].unique()))
    plt.figure(figsize=(10, 10))
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

    plt.title('Robot Trajectory with Collision Highlighting')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    plt.grid()
    plt.show()

def UpdateDf(df, intervals):
    df_updated = df.copy()
    df_updated['Congestion'] = pd.NA
    for i in df['Robot'].unique():
        df_new = get_congestion_coordinates(df, congestion_detection(i, intervals), intervals, i)
        #print(df_new)

        df_updated = pd.merge(df_updated, df_new[['Robot', 'Trajectory', 'Timestep', 'X', 'Y', 'Congestion', 'Game']], on=['Robot', 'Trajectory', 'Timestep', 'X', 'Y', 'Game'], how='left')
        #print(df_updated[df_updated['Robot'] == 'F01'])

        df_updated['Congestion'] = df_updated['Congestion_x'].combine_first(df_updated['Congestion_y'])
        df_updated.drop(columns=['Congestion_x', 'Congestion_y'], inplace=True)
    return df_updated

def plot_all_trajectories(df):
    # Create a figure to plot all the robot trajectories
    plt.figure(figsize=(10, 6))
    
    # Loop through each unique robot in the DataFrame
    for robot in df['Robot'].unique():
        # Filter the DataFrame for the current robot
        robot_data = df[df['Robot'] == robot]
        
        # Plot the X and Y coordinates as a trajectory
        plt.plot(robot_data['X'], robot_data['Y'], linestyle='-', label=f'Robot {robot}')
    
    # Adding labels and title
    plt.title('Trajectories of All Robots')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    
    # Show the plot
    plt.grid(True)
    plt.show()

file_path = './results/cluster_CPFA_r16_tag128_8by8_quard_arena_0_iAntDroppedTrajData1.txt'

second = 3 * 32 
df = parse_trajectory_file(file_path, 0.03125)
df = df[df['Timestep'] >= 2.000]


#plot_all_trajectories(df)
#print(df)
df = UpdateDf(df, second)
#plot_robot_trajectory_with_collisions(df)

#print(Congestion_df)
#use get_collision_coordinates function if you want a specific bot being plotted
#Congestion_bot = get_congestion_coordinates(df, congestion_detection("F11", second), second, "F11")
#plot_robot_trajectory_with_collisions(Congestion_bot)

#print(Congestion_df.head())
#print(Congestion_df.tail())
df['Velocity'] = df.groupby(['Robot', 'Trajectory', 'Game']).apply(
    lambda group: ((group['X'].diff() ** 2 + group['Y'].diff() ** 2) ** 0.5) / 0.03125
).reset_index(level=['Robot', 'Trajectory', 'Game'], drop=True)
df['Velocity'].fillna(0, inplace=True)
df['Congestion'] = df['Congestion'].astype(int)
radius=1
df['Congestion'] = df.apply(
        lambda row: row['Congestion'] if np.sqrt(row['X'] ** 2 + row['Y'] ** 2) <= radius
                  else 0, axis=1)
#Congestion_df = df[df['Congestion'] == 1]
#print(Congestion_df)
print(df)

#features = ['Velocity', 'X', 'Y']  # or other relevant columns
#target = 'Congestion'

#X = df[features]
#y = df[target]

#X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
#model = RandomForestClassifier()
#model.fit(X_train, y_train)
# Make predictions on the test set
#_pred = model.predict(X_test)

# Evaluate the model's performance
#accuracy = accuracy_score(y_test, y_pred)
#print(f'Accuracy: {accuracy:.2f}')

# Get a classification report
#print(classification_report(y_test, y_pred))
#new_df = df[['Robot', 'Trajectory', 'X', 'Y','Timestep', 'Velocity', 'Game']]
#X_new = df[['Velocity', 'X', 'Y']]
#y_pred_new = model.predict(X_new)
#new_df['Congestion'] = y_pred_new

#print(new_df)
plot_robot_trajectory_with_collisions(df)


#with open('./source/CPFA/congestion_model.pkl', 'wb') as file:
#    pickle.dump(model, file)


output_file_path = "./source/CPFA/parsed_trajectory_data.csv"

#Congestion_df.to_csv(output_file_path, index=False, mode='w')







#get density(Count of Robots nearby at that time step),  apply it to CPFA
#CPFA ADD if statement with a radius around the circle, if robot is within that area run machine modle, drop it