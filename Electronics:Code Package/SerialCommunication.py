import tkinter as tk
import serial
import threading
import csv
import time

# Initialize the serial connections to the HC-05 modules
ser_arm = serial.Serial('COM11', 9600)  # Ensure COM11 is the correct outgoing COM port for the arm Arduino
ser_base = serial.Serial('COM8', 9600)  # Ensure COM8 is the correct outgoing COM port for the base Arduino

recording = False
csv_file = None
csv_writer = None

#####################
### Arm Functions ###
#####################

def fire_solenoids():
    ser_arm.write(b"Fire Solenoids\n")
    return

def send_linear_actuator_pos():
    linear_actuator_pos = entry_linear_actuator.get()
    if linear_actuator_pos.lower() == 'exit':
        ser_arm.close()
        root.quit()
    else:
        data = f"Linear Actuator Position (mm): {linear_actuator_pos}\n"
        ser_arm.write(data.encode('utf-8'))

def send_boomerang_speed():
    boomerang_speed = entry_boomerang.get()
    if boomerang_speed.lower() == 'exit':
        ser_arm.close()
        root.quit()
    else:
        data = f"Brushed Motor Speed: {boomerang_speed}\n"
        ser_arm.write(data.encode('utf-8'))

def reset_encoder():
    ser_arm.write(b"Reset Encoder\n")

def send_track_encoder():
    release_angle = entry_release_angle.get()
    target_rpm = entry_target_rpm.get()
    data = f"Track Encoder Release Angle: {release_angle} Target RPM: {target_rpm}\n"
    ser_arm.write(data.encode('utf-8'))

# Function to read from the arm serial port and update the GUI
def read_from_serial_arm():
    global recording, csv_writer
    while True:
        if ser_arm.in_waiting > 0:
            data = ser_arm.readline().decode('utf-8').strip()
            text_serial_output_arm.insert(tk.END, data + '\n')
            text_serial_output_arm.see(tk.END)
            if recording and csv_writer:
                csv_writer.writerow([time.time(), "arm", data])

#######################
### Base Functions ###
#######################

def send_yaw_pos():
    yaw_pos = entry_yaw.get()
    if yaw_pos.lower() == 'exit':
        root.quit()
    else:
        data = f"Yaw Position: {yaw_pos}\n"
        ser_base.write(data.encode('utf-8'))

def send_brushless_speed():
    brushless_speed = entry_brushless.get()
    duration = entry_duration.get()
    steps = entry_steps.get()
    if brushless_speed.lower() == 'exit':
        root.quit()
    else:
        data = f"Brushless Motor Speed: {brushless_speed}\n"
        ser_base.write(data.encode('utf-8'))

# Function to read from the base serial port and update the GUI
def read_from_serial_base():
    global recording, csv_writer
    while True:
        if ser_base.in_waiting > 0:
            data = ser_base.readline().decode('utf-8').strip()
            text_serial_output_base.insert(tk.END, data + '\n')
            text_serial_output_base.see(tk.END)
            if recording and csv_writer:
                csv_writer.writerow([time.time(), "base", data])

# Function to start recording data to a CSV file
def start_recording():
    global recording, csv_file, csv_writer
    filename = entry_filename.get()
    if filename:
        filename = filename + '.csv'
    else:
        filename = 'recorded_data.csv'
    csv_file = open(filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Timestamp', 'Source', 'Data'])
    recording = True

# Function to stop recording data
def stop_recording():
    global recording, csv_file
    recording = False
    if csv_file:
        csv_file.close()
        csv_file = None

# Setup the GUI
root = tk.Tk()
root.title("Position Input")

frame_left = tk.Frame(root)
frame_left.pack(side=tk.LEFT, padx=10, pady=10)

frame_right = tk.Frame(root)
frame_right.pack(side=tk.LEFT, padx=10, pady=10)

label_linear_actuator = tk.Label(frame_left, text="Linear Actuator Position (mm):")
label_linear_actuator.pack()

entry_linear_actuator = tk.Entry(frame_left)
entry_linear_actuator.pack()

button_linear_actuator = tk.Button(frame_left, text="Send Linear Actuator Position", command=send_linear_actuator_pos)
button_linear_actuator.pack()

# Add a new input for Boomerang Motor Speed
label_boomerang = tk.Label(frame_left, text="Boomerang Motor Speed:")
label_boomerang.pack()

entry_boomerang = tk.Entry(frame_left)
entry_boomerang.pack()

button_boomerang = tk.Button(frame_left, text="Send Boomerang Motor Speed", command=send_boomerang_speed)
button_boomerang.pack()

# Add a new input for Yaw Position
label_yaw = tk.Label(frame_left, text="Yaw Position:")
label_yaw.pack()

entry_yaw = tk.Entry(frame_left)
entry_yaw.pack()

button_yaw = tk.Button(frame_left, text="Send Yaw Position", command=send_yaw_pos)
button_yaw.pack()

# Add a new input for Arm Motor Speed
label_brushless = tk.Label(frame_right, text="Arm Motor Speed:")
label_brushless.pack()

entry_brushless = tk.Entry(frame_right)
entry_brushless.pack()

button_brushless = tk.Button(frame_right, text="Send Arm Motor Speed", command=send_brushless_speed)
button_brushless.pack()

# Add inputs for Release Angle and Target RPM
label_release_angle = tk.Label(frame_right, text="Release Angle:")
label_release_angle.pack()

entry_release_angle = tk.Entry(frame_right)
entry_release_angle.pack()

label_target_rpm = tk.Label(frame_right, text="Target RPM:")
label_target_rpm.pack()

entry_target_rpm = tk.Entry(frame_right)
entry_target_rpm.pack()

button_track_encoder = tk.Button(frame_right, text="Track Encoder", command=send_track_encoder)
button_track_encoder.pack()

# Add a button to reset the encoder
button_reset_encoder = tk.Button(frame_right, text="Reset Encoder", command=reset_encoder)
button_reset_encoder.pack()

# Add a button to fire the solenoids
button_fire_solenoids = tk.Button(frame_right, text="Fire Solenoids", command=fire_solenoids)
button_fire_solenoids.pack()

# Add text widgets to display serial output from both Arduinos
label_serial_output_arm = tk.Label(root, text="Arm Serial Output:")
label_serial_output_arm.pack()

text_serial_output_arm = tk.Text(root, height=10, width=50)
text_serial_output_arm.pack(side=tk.LEFT, padx=10, pady=10)

label_serial_output_base = tk.Label(root, text="Base Serial Output:")
label_serial_output_base.pack()

text_serial_output_base = tk.Text(root, height=10, width=50)
text_serial_output_base.pack(side=tk.RIGHT, padx=10, pady=10)

# Center align the labels
label_serial_output_arm.pack(pady=5)
label_serial_output_base.pack(pady=5)

# Add a text box to enter the CSV filename
label_filename = tk.Label(root, text="CSV Filename:")
label_filename.pack()

entry_filename = tk.Entry(root)
entry_filename.pack()

# Add start and stop recording buttons
button_start_recording = tk.Button(root, text="Start Recording", command=start_recording)
button_start_recording.pack()

button_stop_recording = tk.Button(root, text="Stop Recording", command=stop_recording)
button_stop_recording.pack()

# Place the text input and buttons below the text widgets
label_filename.pack(pady=5)
entry_filename.pack(pady=5)
button_start_recording.pack(pady=5)
button_stop_recording.pack(pady=5)

# Start separate threads to read from each serial port
serial_thread_arm = threading.Thread(target=read_from_serial_arm)
serial_thread_arm.daemon = True
serial_thread_arm.start()

serial_thread_base = threading.Thread(target=read_from_serial_base)
serial_thread_base.daemon = True
serial_thread_base.start()

root.mainloop()

ser_arm.close()
ser_base.close()
