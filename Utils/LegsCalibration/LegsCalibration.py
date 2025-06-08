import time
import tkinter as tk
from tkinter import ttk

import serial

# === Serial Setup ===
SERIAL_PORT = "COM8"
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"Error: Could not open serial port {SERIAL_PORT}")
    ser = None

root = tk.Tk()
root.title("Leg Calibration GUI")
root.geometry("1000x600")

# === Constants ===
JOINT_NAMES = ["Knee", "Hip-Leg", "Hip-Body"]
JOINT_RANGES = [(0, 150), (-247, -43), (-43, 37)]
NUM_LEGS = 4
NUM_JOINTS = 3

# === Storage for UI elements ===
sliders = [[None for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]
offsets = [[None for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]


# === Send data ===
def send_all_leg_data():
    values = []
    for leg in range(NUM_LEGS):
        for j in range(NUM_JOINTS):
            try:
                angle = sliders[leg][j].get()
                offset = float(offsets[leg][j].get())
                total = int(angle + offset)
            except ValueError:
                total = int(sliders[leg][j].get())
            values.append(str(total))
    command = " ".join(values) + "\n"
    if ser:
        ser.write(command.encode("utf-8"))
    print("Sent:", command.strip())


# === Callback for slider move ===
def on_slider_change(event=None):
    send_all_leg_data()


# === Create 2x2 Leg Sections ===
for leg in range(NUM_LEGS):
    frame = ttk.LabelFrame(root, text=f"Leg {leg+1}", padding=10)
    row = leg // 2
    col = leg % 2
    frame.grid(row=row, column=col, padx=20, pady=20, sticky="nsew")

    for j in range(NUM_JOINTS):
        ttk.Label(frame, text=JOINT_NAMES[j]).grid(
            row=j, column=0, padx=5, pady=5, sticky="w"
        )

        min_val, max_val = JOINT_RANGES[j]
        slider = ttk.Scale(
            frame,
            from_=min_val,
            to=max_val,
            orient="horizontal",
            length=200,
            command=lambda val, l=leg, jj=j: on_slider_change(),
        )
        slider.set((min_val + max_val) / 2)
        slider.grid(row=j, column=1, padx=5, pady=5)
        sliders[leg][j] = slider

        offset_var = tk.StringVar(value="0")
        offset_entry = ttk.Entry(frame, textvariable=offset_var, width=6)
        offset_entry.grid(row=j, column=2, padx=5, pady=5)
        offsets[leg][j] = offset_entry

        send_button = ttk.Button(frame, text="⏎", width=3, command=send_all_leg_data)
        send_button.grid(row=j, column=3, padx=5)

root.mainloop()

if ser:
    ser.close()
