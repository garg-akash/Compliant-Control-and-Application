import matplotlib.pyplot as plt
import numpy as np
import time

def read_pose_error(filename):
    data = np.loadtxt(filename)
    print("loaded")
    position_error = data[:, :3]
    orientation_error = data[:, 3:]
    return position_error, orientation_error

pre = "/home/akash/catkin_ws/controller_logs/"
filename = 'pose_error.txt'
print("enter")
plt.ion()  # Turn on interactive mode
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

while True:
    try:
        position_error, orientation_error = read_pose_error(pre + filename)
        # extract position errors in x,y,z
        position_error_x = [error[0] for error in position_error]
        position_error_y = [error[1] for error in position_error]
        position_error_z = [error[2] for error in position_error]

        axs[0].clear()
        axs[1].clear()

        # axs[0].plot(position_error, marker='o', linestyle='-', color='b')
        axs[0].plot(position_error_x, marker='o', linestyle='-', color='r', label='Error in X')
        axs[0].plot(position_error_y, marker='o', linestyle='-', color='g', label='Error in Y')
        axs[0].plot(position_error_z, marker='o', linestyle='-', color='b', label='Error in Z')

        axs[0].set_title('Position Error')
        axs[0].set_xlabel('Iteration')
        axs[0].set_ylabel('Error (m)')
        axs[0].grid(True)
        axs[0].legend(loc='best')

        orientation_error_x = [orient[0] for orient in orientation_error]
        orientation_error_y = [orient[1] for orient in orientation_error]
        orientation_error_z = [orient[2] for orient in orientation_error]
        axs[1].plot(orientation_error_x, marker='o', linestyle='-', color='r', label='Error in x')
        axs[1].plot(orientation_error_y, marker='s', linestyle='-', color='g', label='Error in y')
        axs[1].plot(orientation_error_z, marker='^', linestyle='-', color='b', label='Error in z')
        
        axs[1].set_title('Orientation Error')
        axs[1].set_xlabel('Iteration')
        axs[1].set_ylabel('Error')
        axs[1].grid(True)
        axs[1].legend(loc='best')

        plt.pause(1)  # Pause for a second before updating the plot

    except Exception as e:
        print(f"Error: {e}")

    time.sleep(1)  # Sleep for a while before reading the file again
