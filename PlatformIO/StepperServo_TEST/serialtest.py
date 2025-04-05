import serial
import struct
import threading
import time

# Configure the serial port
ser = serial.Serial('/dev/tty.usbmodem1101', 115200)  # Update port and baud rate
if ser.is_open:
    print("Port is open!")
else:
    print("Failed to open port.")

# Global variable for shared value
value_lock = threading.Lock()  # Lock for thread-safe access to the value
value = 0.3

# Function to send data
def send_data():
    global value
    while True:
        # Send value 0.2 for 50 iterations
        for _ in range(50):
            value = 0.2
            # Pack the value as a float (4 bytes)
            data = struct.pack('<f', value)
            # Send sync byte (0xAA) first, then the 4-byte float value
            ser.write(bytes([0xAA]))  # Sync byte
            ser.write(data)  # 4 bytes of float data
            print(f"Sent: {value} as bytes: {[hex(b) for b in [0xAA] + list(data)]}")
            time.sleep(0.1)  # Delay to avoid spamming the serial port

        # Send value 0.4 for 50 iterations
        for _ in range(50):
            value = 0.4
            data = struct.pack('<f', value)
            ser.write(bytes([0xAA]))  # Sync byte
            ser.write(data)  # 4 bytes of float data
            print(f"Sent: {value} as bytes: {[hex(b) for b in [0xAA] + list(data)]}")
            time.sleep(0.1)  # Delay to avoid spamming the serial port

        for _ in range(20):
            value = 0.3
            data = struct.pack('<f', value)
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
            print(f"Received Float: {raw_data}")  # Print unpacked float

# Create and start threads
send_thread = threading.Thread(target=send_data, daemon=True)
receive_thread = threading.Thread(target=receive_data, daemon=True)

send_thread.start()
receive_thread.start()

# Keep the main thread running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
