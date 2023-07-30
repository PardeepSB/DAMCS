import socket
import serial
import time

# FlightGear settings
FG_HOST = 'localhost'  # Change this to the IP of the computer running FlightGear if needed
FG_PORT = 5500  # The port number configured in the FlightGear settings

# COM port settings
COM_PORT = 'COMX'  # Change this to the appropriate COM port name
BAUD_RATE = 9600

print_statement = True

# Create the socket object (outside the main function)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    # Connect to FlightGear
    client_socket.connect((FG_HOST, FG_PORT))
except ConnectionRefusedError:
    print("Connection to FlightGear refused. Make sure FlightGear is running and accepting external commands.")
    exit()

def send_command(command):
    global print_statement

    try:
        # Send the command
        client_socket.sendall(command.encode())

    except Exception as e:
        print("Error:", e)

def main():
    try:
        # Open the COM port
        ser = serial.Serial(COM_PORT, BAUD_RATE)
        print(f"Connected to {COM_PORT} at {BAUD_RATE} baud rate.")

        while True:
            start_time = time.time()  # Record the start time of the loop

            # Read a line of data from the COM port (assuming data format: "aileron,elevator")
            rxBuffer = ser.readline().decode().strip()
            Buf = rxBuffer.split(",")
            Buf = [int(Buf[0]), int(Buf[1])]
            
            # Split the data into aileron and elevator values
            aileron = Buf[0] / 90
            elevator = Buf[1] / 90
            print(Buf)

            # Validate input values to stay within the range
            aileron = max(-1.0, min(1.0, aileron))
            elevator = max(-1.0, min(1.0, elevator))

            # Construct the command to send to FlightGear
            command = f"set /controls/flight/aileron {aileron}\r\nset /controls/flight/elevator {elevator}\r\n"

            # Send the command to FlightGear
            send_command(command)

            # Calculate the time taken for the processing so far
            elapsed_time = time.time() - start_time

            # Calculate the remaining time for this iteration (target 50Hz = 1/50 = 0.02 seconds)
            remaining_time = 0.02 - elapsed_time

            # Add a delay to achieve the desired 50Hz frequency
            if remaining_time > 0:
                time.sleep(remaining_time)

    except serial.SerialException as se:
        print(f"Error: {se}. Make sure the COM port ({COM_PORT}) is available and configured correctly.")
    except ValueError:
        print("Invalid data received from the COM port. Data format should be 'aileron,elevator'.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Close the COM port and the socket connection when exiting the loop
        ser.close()
        client_socket.close()

if __name__ == "__main__":
    main()
