import serial
import struct
import threading
import time
import tkinter as tk

# Configure the serial port
ser = serial.Serial('/dev/tty.usbmodem2101', 115200)  # Update port and baud rate
if ser.is_open:
    print("Port is open!")
else:
    print("Failed to open port.")

# Global variable for shared value
value_lock = threading.Lock()  # Lock for thread-safe access to the value
value = 0.30

# Function to send data
def send_data():
    global value
    while True:
        with value_lock:
            # Pack the value as a float (4 bytes)
            data = struct.pack('<f', value)
        # Send sync byte (0xAA) first, then the 4-byte float value
        ser.write(bytes([0xAA]))  # Sync byte
        ser.write(data)  # 4 bytes of float data
        print(f"Sent: {value} as bytes: {[hex(b) for b in [0xAA] + list(data)]}")
        time.sleep(0.1)  # Delay to avoid spamming the serial port

# Function to receive data
def receive_data():
    while True:
        if ser.in_waiting >= 4:  # Wait until 4 bytes are available
            raw_data = ser.read(4)  # Read 4 bytes
            received_value = struct.unpack('<f', raw_data)[0]  # Convert to float
            print(f"Received Float: {received_value}")  # Print unpacked float

# Function to update the value from the slider
def update_value(new_value):
    global value
    with value_lock:
        value = float(new_value)

# Create Tkinter GUI
def create_gui():
    root = tk.Tk()
    root.title("Serial Value Slider")

    tk.Label(root, text="Select Value").pack(pady=10)

    # Create a larger slider
    slider = tk.Scale(
        root,
        from_=0,
        to=0.6,
        resolution=0.01,
        orient=tk.HORIZONTAL,
        command=update_value,
        length=400,  # Adjust the length of the slider
        sliderrelief=tk.GROOVE,  # Style the slider
        width=20  # Adjust the thickness of the slider handle
    )
    slider.set(value)  # Set default value
    slider.pack(pady=10)

    root.mainloop()

# Create and start threads
send_thread = threading.Thread(target=send_data, daemon=True)
receive_thread = threading.Thread(target=receive_data, daemon=True)

send_thread.start()
receive_thread.start()

# Start the Tkinter GUI in the main thread
create_gui()

# Keep the program running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
