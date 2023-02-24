#!/usr/bin/env python3

import matplotlib.pyplot as plt
import csv
from pathlib import Path


def plot_motion_profile(csv_path):
    '''
    This function plots the values from a csv file with time_steps, position, velocity and acceleration.
    It plots two graphs. One with 3 subplots and the other with all three curves superimposed on each other
    :param csv_path: path to the trajectories.csv file
    :return:
    '''
    # Initialize lists for time, position, velocity, and acceleration
    time = []
    position = []
    velocity = []
    acceleration = []

    # Use a context manager to open the CSV file and read the data
    with csv_path.open('r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            time.append(float(row[0]))
            position.append(float(row[1]))
            velocity.append(float(row[2]))
            acceleration.append(float(row[3]))

    # Create a single subplot for position, velocity, and acceleration
    fig_all, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(12, 4))

    ax.plot(time, position, color='g', marker='o', label="Position")
    ax.plot(time, velocity, color='b', marker='o', label="Velocity")
    ax.plot(time, acceleration, color='r', marker='o', label="Acceleration")
    ax.set_title('Motion Profile')
    ax.set_ylabel('Value (steps)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid()

    ax1.plot(time, position, color='g', marker='o', label="")
    ax1.set_title('Position vs Time')
    ax1.set_ylabel('Position (steps)')
    ax1.set_xlabel('Time (s)')
    ax1.grid()

    ax2.plot(time, velocity, color='b', marker='o', label="")
    ax2.set_title('Velocity vs Time')
    ax2.set_ylabel('Velocity (steps)')
    ax2.set_xlabel('Time (s)')
    ax2.grid()

    ax3.plot(time, acceleration, color='r', marker='o', label="")
    ax3.set_title('Acceleration vs Time')
    ax3.set_ylabel('Acceleration (steps)')
    ax3.set_xlabel('Time (s)')
    ax3.grid()

    plt.show()


def main():
    csv_path = Path('../data/trajectories.csv')
    plot_motion_profile(csv_path)


if __name__ == '__main__':
    main()

