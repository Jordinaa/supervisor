'''
saves data as an svg file
'''


import csv
import matplotlib.pyplot as plt

def plot_data_from_csv(filename):
    # Read the data from the CSV file
    time_data = []
    roll_data = []
    pitch_data = []
    yaw_data = []
    
    with open(filename, "r") as f:
        csv_reader = csv.reader(f)
        next(csv_reader) # skip the header row
        for row in csv_reader:
            time_data.append(float(row[0]))
            roll_data.append(float(row[1]))
            pitch_data.append(float(row[2]))
            yaw_data.append(float(row[3]))

    # Create a plot
    fig, ax = plt.subplots()
    ax.plot(time_data, roll_data, label="Roll")
    ax.plot(time_data, pitch_data, label="Pitch")
    ax.plot(time_data, yaw_data, label="Yaw")
    ax.legend()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Drone Attitude")
    
    # Save the plot as an SVG file
    fig.savefig("drone_attitude.svg")

plot_data_from_csv("/home/taranto/catkin_ws/src/offboard_py/data/drone_data.csv")