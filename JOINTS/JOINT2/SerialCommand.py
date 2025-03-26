import serial
import threading
import time
import tkinter as tk

import serial.tools.list_ports

# List all available serial ports
ports = list(serial.tools.list_ports.comports())

# Check if there are any available USB serial ports
found = False
for port in ports:
    if 'USB' in port.description:  # Looking for ports related to USB devices
        try:
            ser = serial.Serial(port.device, 115200)  # Open the port with the found USB serial
            print(f"Connected to {port.device}")
            found = True
            break  # Exit the loop once we find a working port
        except serial.SerialException:
            print(f"Failed to open port {port.device}")
            continue

if not found:
    print("No USB serial devices found.")
else:
    # You can now interact with the serial device
    if ser.is_open:
        print("Port is open!")
    else:
        print("Failed to open port.")

# Global variable for shared value
value_lock = threading.Lock()  # Lock for thread-safe access to the value
value = 180
delay = 0.05  # 50ms

# Function to send data
def send_data(log_widget):
    global value
    while True:
        with value_lock:
            data = int(value)
        message = str(int(data)) + '\n'
        ser.write(message.encode())
        log_widget.insert(tk.END, f"Sent: {data}\n") 
        log_widget.see(tk.END)  # Auto-scroll to the bottom
        time.sleep(delay)  # Delay to avoid spamming the serial port

# Function to receive data
def receive_data(log_widget):
    while True:
        if ser.in_waiting > 0:  # Check if there is data available
            data = ser.readline()  # Read one line of data
            decoded_data = data.decode('utf-8').strip()
            log_widget.insert(tk.END, f"Received: {decoded_data}\n")
            log_widget.see(tk.END)  # Auto-scroll to the bottom

# Function to update the value from the slider
def update_value(new_value):
    global value
    new_value = int(new_value)  # Ensure the value is an integer

    with value_lock:
        value = new_value

# Function to reset the value to zero
def reset_value(slider):
    global value
    with value_lock:
        value = 180
    slider.set(value)  # Update the slider position

# Create Tkinter GUI
def create_gui():
    root = tk.Tk()
    root.title("Joint Theta Control")

    # Make the window always on top
    root.attributes("-topmost", 1)

    # Raise the window to the top of the stack
    root.lift()

    tk.Label(root, text="Select Theta Value").pack(pady=10)

    # Create a slider
    slider = tk.Scale(
        root,
        from_=0,
        to=360,
        resolution=1,
        orient=tk.HORIZONTAL,
        command=update_value,
        length=400,  # Adjust the length of the slider
        sliderrelief=tk.GROOVE,  # Style the slider
        width=20  # Adjust the thickness of the slider handle
    )
    slider.set(value)  # Set default value
    slider.pack(pady=10)

    # Add a button to reset the value to zero
    reset_button = tk.Button(
        root,
        text="Reset to 180",
        command=lambda: reset_value(slider),
        bg="lightgray",
        padx=10,
        pady=5
    )
    reset_button.pack(pady=10)

    # Create text boxes for send and receive logs
    send_label = tk.Label(root, text="Send Log")
    send_label.pack(pady=5)

    send_log = tk.Text(root, height=20, width=50, wrap=tk.WORD)
    send_log.pack(pady=5)

    receive_label = tk.Label(root, text="Receive Log")
    receive_label.pack(pady=5)

    receive_log = tk.Text(root, height=20, width=50, wrap=tk.WORD)
    receive_log.pack(pady=5)

    # Start threads with logging widgets
    send_thread = threading.Thread(target=send_data, args=(send_log,), daemon=True)
    receive_thread = threading.Thread(target=receive_data, args=(receive_log,), daemon=True)

    send_thread.start()
    receive_thread.start()

    root.mainloop()

# Start the Tkinter GUI in the main thread
create_gui()
