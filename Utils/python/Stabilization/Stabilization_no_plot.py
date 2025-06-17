import multiprocessing as mp
import os
import sys
import time
import tkinter as tk
from tkinter import ttk

# Add paths
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)
libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from kinematics import inverse_kinematics
from pid_leg_stabilizer import PIDLegStabilizer
from serial_comm import SerialComm

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND


def pid_process(shared_vars, stop_event):
    imu_uc = SerialComm(SERIAL_PORT_REC, BAUD_RATE_REC)
    motor_uc = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND)
    pid_leg = PIDLegStabilizer()
    pid_leg.set_setpoint(0.0, 0.0)
    foot_positions = [[-95, -195], [-95, -195], [95, -195], [95, -195]]

    loop_counter = 0
    last_print_time = time.time()

    # Initialize next loop target time
    freq = shared_vars.freq if shared_vars.freq > 0 else 50
    dt = 1.0 / freq
    next_time = time.perf_counter()

    while not stop_event.is_set():
        # Update freq and dt every loop for dynamic freq changes
        freq = shared_vars.freq if shared_vars.freq > 0 else 50
        dt = 1.0 / freq

        # Read PID params from shared_vars (no blocking)
        pid_leg.kp_pitch = shared_vars.kp_pitch
        pid_leg.ki_pitch = shared_vars.ki_pitch
        pid_leg.kd_pitch = shared_vars.kd_pitch
        pid_leg.kp_roll = shared_vars.kp_roll
        pid_leg.ki_roll = shared_vars.ki_roll
        pid_leg.kd_roll = shared_vars.kd_roll

        # Read IMU
        try:
            imu_uc.ser.reset_input_buffer()
            time.sleep(0.002)
            ypr_data = imu_uc.read_line().strip()
            if ypr_data.startswith("<") and ypr_data.endswith(">"):
                yaw, pitch, roll = map(float, ypr_data[1:-1].split(","))
            else:
                raise ValueError("Bad IMU data")
        except Exception as e:
            print(f"[WARN] IMU read error: {e}")
            # Wait until next frame time to maintain frequency even on error
            next_time += dt
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        # Compute PID
        deltas = pid_leg.compute_leg_deltas(pitch, roll, dt)
        angle_values = []
        ik_failed = False

        for leg in range(NUM_LEGS):
            x, y = foot_positions[leg]
            new_y = y + deltas[leg]
            foot_positions[leg][1] = new_y

            try:
                knee, hip = inverse_kinematics(x, new_y)
                if knee is None or hip is None:
                    raise ValueError("IK failed")
            except Exception as e:
                print(f"[WARN] IK leg {leg+1} failed: {e}")
                ik_failed = True
                break

            angle_values.extend([str(int(knee)), str(int(hip)), "0"])

        if not ik_failed:
            command = "<" + ",".join(angle_values) + ">"
            try:
                motor_uc.write_line(command)
            except Exception as e:
                print(f"[ERROR] Send failed: {e}")

        # Loop frequency debug
        loop_counter += 1
        now = time.time()
        if now - last_print_time >= 1.0:
            print(
                f"[FREQ] PID Process running at {loop_counter / (now - last_print_time):.2f} Hz"
            )
            loop_counter = 0
            last_print_time = now

        # Wait until the next scheduled time
        next_time += dt
        sleep_time = next_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            # We're behind schedule, can optionally log or just continue without sleep
            next_time = time.perf_counter()

    imu_uc.close()
    motor_uc.close()


# === Main GUI ===
def start_gui():
    root = tk.Tk()
    root.title("PID Tuner")

    # Create manager and shared namespace for PID params
    manager = mp.Manager()
    shared_vars = manager.Namespace()
    # Initialize shared variables
    shared_vars.kp_pitch = 0.0
    shared_vars.ki_pitch = 0.0
    shared_vars.kd_pitch = 0.0
    shared_vars.kp_roll = 0.0
    shared_vars.ki_roll = 0.0
    shared_vars.kd_roll = 0.0
    shared_vars.freq = 50.0

    stop_event = mp.Event()

    # Start PID process
    pid_proc = mp.Process(target=pid_process, args=(shared_vars, stop_event))
    pid_proc.start()

    # Variables linked to sliders
    kp_pitch = tk.DoubleVar(value=shared_vars.kp_pitch)
    ki_pitch = tk.DoubleVar(value=shared_vars.ki_pitch)
    kd_pitch = tk.DoubleVar(value=shared_vars.kd_pitch)
    kp_roll = tk.DoubleVar(value=shared_vars.kp_roll)
    ki_roll = tk.DoubleVar(value=shared_vars.ki_roll)
    kd_roll = tk.DoubleVar(value=shared_vars.kd_roll)
    freq_var = tk.StringVar(value=str(shared_vars.freq))

    def create_slider(parent, label, variable, row, max_val=5.0):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        ttk.Scale(
            parent, variable=variable, from_=0.0, to=max_val, orient="horizontal"
        ).grid(row=row, column=1, sticky="ew")

    frame = ttk.Frame(root, padding="10")
    frame.grid(row=0, column=0, sticky="nsew")
    root.columnconfigure(0, weight=1)

    create_slider(frame, "Kp Pitch", kp_pitch, 0)
    create_slider(frame, "Ki Pitch", ki_pitch, 1, max_val=1.0)
    create_slider(frame, "Kd Pitch", kd_pitch, 2, max_val=1.0)
    create_slider(frame, "Kp Roll", kp_roll, 3)
    create_slider(frame, "Ki Roll", ki_roll, 4, max_val=1.0)
    create_slider(frame, "Kd Roll", kd_roll, 5, max_val=1.0)

    ttk.Label(frame, text="Frequency (Hz)").grid(row=6, column=0, sticky="w")
    freq_entry = ttk.Entry(frame, textvariable=freq_var, width=10)
    freq_entry.grid(row=6, column=1, sticky="w")

    def update_shared_vars():
        try:
            shared_vars.kp_pitch = kp_pitch.get()
            shared_vars.ki_pitch = ki_pitch.get()
            shared_vars.kd_pitch = kd_pitch.get()
            shared_vars.kp_roll = kp_roll.get()
            shared_vars.ki_roll = ki_roll.get()
            shared_vars.kd_roll = kd_roll.get()
            shared_vars.freq = float(freq_var.get())
        except Exception:
            pass
        root.after(100, update_shared_vars)

    root.after(100, update_shared_vars)

    def on_close():
        stop_event.set()
        pid_proc.join()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    mp.set_start_method("spawn")  # For cross-platform compatibility
    start_gui()
