import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import os
import matplotlib.pyplot as plt
import sys

# Initialize the chain
my_chain = ikpy.chain.Chain.from_urdf_file("/Users/alexandermartinez/Desktop/IGEN/Injection/PYTHON/fullassembly5.urdf")

# Function to update the plot
def update_plot(x, y, z):
    target_position = [x, y, z]
    angles = my_chain.inverse_kinematics(target_position)
    print("The angles of each joints are : ", angles)
    
    fig, ax = plot_utils.init_3d_figure()
    my_chain.plot(angles, ax, target=target_position)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    plt.draw()
    plt.pause(0.001)

# Default target position
x, y, z = 0.1, 0.2, 0.3

# Function to handle user input and update the plot
def handle_input():
    global x, y, z
    plt.ion()  # Turn on interactive mode
    while True:
        user_input = input("Enter axis and value (e.g., 'x 0.1') or 'exit' to quit: ")
        if user_input.lower() == 'exit':
            plt.ioff()  # Turn off interactive mode
            break
        try:
            axis, value = user_input.split()
            value = float(value)
            if axis == 'x':
                x = value
            elif axis == 'y':
                y = value
            elif axis == 'z':
                z = value
            else:
                print("Invalid axis. Use 'x', 'y', or 'z'.")
                continue
            update_plot(x, y, z)
        except ValueError:
            print("Invalid input. Please enter in the format 'axis value'.")

# Start handling user input
handle_input()