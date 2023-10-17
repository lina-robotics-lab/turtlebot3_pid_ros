import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_runs():
    dfs = []
    dir = '/home/mht/turtlebot3_ws/src/turtlebot3_pid_pos/turtlebot3_pid_pos/data/1120_to_1140' # replace with your data directory
    
    # a. If you would like to plot all the data
    files = os.listdir(dir)
    
    # b. If you only want to plot several specific runs
    
    # files = [
    #         'data_2023-10-13_11-29-05_DIST_0.2_P_3.5_I_0.1_D_0.2.csv',
    #         'data_2023-10-13_11-29-29_DIST_0.3_P_3.5_I_0.1_D_0.2.csv'
    #         # add more here
    #          ]
    for file in files:
        if file.endswith('csv'):
            df = pd.read_csv(os.path.join(dir, file))
            df['run'] = file[16:-4]
            dfs.append(df)
        total_df = pd.concat(dfs, ignore_index=True)

    fig = plt.figure()
    sns.lineplot(total_df, x='time', y='pos_error', hue='run')
    plt.show()

plot_runs()
    