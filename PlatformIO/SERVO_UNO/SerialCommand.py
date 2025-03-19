import serial
import threading
import time
import tkinter as tk

# Configure the serial port
ser = serial.Serial('/dev/tty.usbserial-1110', 115200)  # Update port and baud rate
if ser.is_open:
    print("Port is open!")
else:
    print("Failed to open port.")

# Global variable for shared value
value_lock = threading.Lock()  # Lock for thread-safe access to the value
value = 0
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
    new_value = float(new_value)  # Ensure the value is a float

    # Check if the value is 10 or greater
    if new_value >= 10:
        new_value *= 10  # Multiply by 10 if the value is 10 or greater

    with value_lock:
        value = new_value


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
        from_=-255,
        to=255,
        resolution=1,
        orient=tk.HORIZONTAL,
        command=update_value,
        length=400,  # Adjust the length of the slider
        sliderrelief=tk.GROOVE,  # Style the slider
        width=20  # Adjust the thickness of the slider handle
    )
    slider.set(value)  # Set default value
    slider.pack(pady=10)

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
