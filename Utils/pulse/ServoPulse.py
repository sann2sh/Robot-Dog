import os
import sys
import time
import tkinter as tk
from tkinter import ttk

import serial

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../.."))
sys.path.append(config_dir)

# Import the config module
from config import BAUD_RATE
from config import SERIAL_PORT

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
except serial.SerialException:
    print(f"Error: Could not open serial port {SERIAL_PORT}")
    ser = None


def send_raw_pulse(servo_id, pulse):
    pulse = int(float(pulse))
    pulse = max(0, min(1000, pulse))
    if ser:
        command = f"{servo_id}:{pulse}\n"
        ser.write(command.encode("utf-8"))
        print(f"Sent: {command.strip()}")
    value_labels[servo_id].config(text=str(pulse))
    if entry_boxes[servo_id].get() != str(pulse):
        entry_boxes[servo_id].delete(0, tk.END)
        entry_boxes[servo_id].insert(0, str(pulse))


def slider_changed(val, idx):
    pulse = int(float(val))
    servo_vars[idx].set(pulse)
    send_raw_pulse(idx, pulse)


def step_pulse(idx, direction):
    current_val = servo_vars[idx].get()
    new_val = max(0, min(1000, current_val + direction))
    servo_vars[idx].set(new_val)
    sliders[idx].set(new_val)
    send_raw_pulse(idx, new_val)


def entry_updated(idx):
    try:
        val = int(entry_boxes[idx].get())
        val = max(0, min(1000, val))
        servo_vars[idx].set(val)
        sliders[idx].set(val)
        send_raw_pulse(idx, val)
    except ValueError:
        pass


# Create GUI
root = tk.Tk()
root.title("Servo Pulse Calibrator")
root.geometry("1000x500")  # Double-size window

initial_value = 425
servo_vars = [tk.IntVar(value=initial_value) for _ in range(3)]
value_labels = []
entry_boxes = []
sliders = []

style = ttk.Style()
style.configure("TLabel", font=("Arial", 16))
style.configure("TButton", font=("Arial", 14))
style.configure("TEntry", font=("Arial", 14))

for i in range(3):
    # Servo label
    ttk.Label(root, text=f"Servo {i}").grid(
        row=i, column=0, padx=20, pady=25, sticky="w"
    )

    # Slider
    slider = ttk.Scale(
        root,
        from_=0,
        to=1000,
        orient="horizontal",
        variable=servo_vars[i],
        length=500,
        command=lambda val, idx=i: slider_changed(val, idx),
    )
    slider.grid(row=i, column=1, padx=10, pady=25)
    sliders.append(slider)

    # Value display
    val_label = ttk.Label(root, text=str(initial_value), width=6)
    val_label.grid(row=i, column=2)
    value_labels.append(val_label)

    # Entry box
    entry = ttk.Entry(root, width=8, font=("Arial", 16))
    entry.insert(0, str(initial_value))
    entry.grid(row=i, column=3)
    entry_boxes.append(entry)

    # Set button
    entry_button = ttk.Button(
        root, text="Set", width=5, command=lambda idx=i: entry_updated(idx)
    )
    entry_button.grid(row=i, column=4, padx=5)

    # Up/Down buttons
    ttk.Button(root, text="▲", width=3, command=lambda idx=i: step_pulse(idx, +1)).grid(
        row=i, column=5, padx=5
    )
    ttk.Button(root, text="▼", width=3, command=lambda idx=i: step_pulse(idx, -1)).grid(
        row=i, column=6, padx=5
    )

    # Send initial pulse
    send_raw_pulse(i, initial_value)

root.mainloop()

# Close serial on exit
if ser:
    ser.close()
