import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Define the serial port and baud rate
serial_port = 'COMX'  # Replace 'X' with the actual COM port number
baud_rate = 9600

# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate)

# Create a figure and axis for plotting
fig, ax = plt.subplots()
line1, = ax.plot([], [], 'r-', label='Pitch')   # Line for pitch, displayed in red
line2, = ax.plot([], [], 'b-', label='Roll')    # Line for roll, displayed in blue
ax.set_xlim(0, 50)
ax.set_ylim(-90, 90)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Real-Time Data Plot')
ax.legend()

# Initialize empty lists to store data
x_values = []
y_values_data1 = []
y_values_data2 = []
start_time = time.time()
paused = False
pause_start_time = 0

# Create an initial text object with empty text
text = ax.text(0.05, 0.95, "", transform=ax.transAxes, fontsize=10, ha='left', va='top', color='red', weight='bold')

# Function to update the plot data
def update_plot(frame):
    global paused, pause_start_time

    if not paused:
        data = ser.readline().decode().strip()
        data = data.split(",")
        if len(data) == 2:
            try:
                value1 = int(data[0])
                value2 = int(data[1])

                current_time = time.time()
                if not paused:
                    elapsed_time = current_time - start_time
                else:
                    elapsed_time = pause_start_time - start_time

                x_values.append(elapsed_time)
                y_values_data1.append(value1)
                y_values_data2.append(value2)

                # Adjust x-axis limits dynamically based on the latest data points
                if elapsed_time > 50:
                    ax.set_xlim(elapsed_time - 50, elapsed_time)
                else:
                    ax.set_xlim(0, 50)

                line1.set_data(x_values, y_values_data1)
                line2.set_data(x_values, y_values_data2)
            except ValueError:
                pass
    
    # Update the text annotation on the plot (top left)
    if paused:
        text.set_text("Press P to unpause")
    else:
        text.set_text("Press P to pause")  # Set the text to an empty string to clear the previous annotation


# Function to handle pause/unpause events
def on_key(event):
    global paused, pause_start_time, start_time
    if event.key == 'p':
        if not paused:
            pause_start_time = time.time()
            paused = True
            print("Plot paused.")
        else:
            start_time += time.time() - pause_start_time
            paused = False
            print("Plot resumed.")

# Attach the key press event to the figure
fig.canvas.mpl_connect('key_press_event', on_key)

# Main function for animation
def animate(frame):
    update_plot(frame)

# Call the animation function to start real-time plotting
ani = animation.FuncAnimation(fig, animate, interval=0)

# Display the plot
plt.show()
