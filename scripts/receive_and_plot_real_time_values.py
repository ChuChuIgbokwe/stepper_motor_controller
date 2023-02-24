#!/usr/bin/env python3
import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re


def run_animation():
    '''
    Creates a socket object, binds it to a specific IP address and port number, listens for incoming connections,
    and receives data in 16-byte chunks. It then parses the data and plots graphs of position, velocity,
    and acceleration over time.
    :return:
    '''
    # Define the IP address and port number to listen on
    IP_ADDRESS = '127.0.0.1'  # Replace with your IP address
    PORT = 8082  # Replace with your desired port number

    # Create a socket object and bind it to the IP address and port number
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((IP_ADDRESS, PORT))

    # Listen for incoming connections
    sock.listen()

    # Accept a connection and receive data in a loop
    conn, addr = sock.accept()
    print(f"Connected to {addr}")

    time_steps = []
    current_position = []
    current_velocity = []
    current_acceleration = []

    # Create an empty buffer list
    buffer = []

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(12, 4))

    # Define the animation function
    def animate(i):
        '''

        :param i:
        :return:
        '''
        try:
            # Receive data in 16-byte chunks (4 bytes for each parameter)
            data = conn.recv(1024)
            if not data:
                anim.event_source.stop()
                return

            # Add the received message to the buffer
            buffer.append(data)
            data_out = buffer.pop(0)

            # Split the data by comma and \n
            data_list = re.split(b'[,\n]', data_out)
            data_list = list(filter(lambda x: x != b'', data_list))  # Remove empty elements

            data_list = [elem.decode() for elem in data_list]

            # If the data is larger than 4 and is a multiple of 4, break it up into chunks of 4 and append it
            if len(data_list) != 4 and len(data_list) % 4 == 0:
                for data_quartile_index in range(0, len(data_list), 4):
                    time_elapsed, position, velocity, acceleration = map(float, data_list[data_quartile_index:
                                                                                          data_quartile_index + 4])
                    time_steps.append(time_elapsed)
                    current_position.append(position)
                    current_velocity.append(velocity)
                    current_acceleration.append(acceleration)
                    data_quartile_index += 4

            # If the data is larger than 4 and is not a multiple of 4, discard the elements greater than a multiple of 4
            if len(data_list) != 4 and len(data_list) % 4 != 0:
                data_list = data_list[:4]
                time_elapsed, position, velocity, acceleration = map(float, data_list)

                time_steps.append(time_elapsed)
                current_position.append(position)
                current_velocity.append(velocity)
                current_acceleration.append(acceleration)

            if len(data_list) == 4:
                print("4 values", len(data_list))
                time_elapsed, position, velocity, acceleration = map(float, data_list)
                time_steps.append(time_elapsed)
                current_position.append(position)
                current_velocity.append(velocity)
                current_acceleration.append(acceleration)

            # Print the variables
            print(f"\nTime elapsed: {time_elapsed}")
            print(f"Current position: {position}")
            print(f"Current velocity: {velocity}")
            print(f"Current acceleration: {acceleration}")

            # Plot the graphs
            ax1.clear()
            ax1.plot(time_steps, current_position)
            ax1.set_title('Position vs Time')
            ax1.set_ylabel('Position (steps)')
            ax1.set_xlabel('Time (s)')
            ax1.grid()

            ax2.clear()
            ax2.plot(time_steps, current_velocity)
            ax2.set_title('Velocity vs Time')
            ax2.set_ylabel('Velocity (steps/s)')
            ax2.set_xlabel('Time (s)')
            ax2.grid()

            ax3.clear()
            ax3.plot(time_steps, current_acceleration)
            ax3.set_title('Acceleration vs Time')
            ax3.set_ylabel('Acceleration (steps/s^2)')
            ax3.set_xlabel('Time (s)')
            ax3.grid()
        except Exception as e:
            print(f"Error: {e}")


    # Create the animation
    anim = animation.FuncAnimation(fig, animate, frames=500, interval=1000, blit=False)
    plt.show()

    # Close the connection and socket
    conn.close()
    sock.close()


if __name__ == "__main__":
    run_animation()
