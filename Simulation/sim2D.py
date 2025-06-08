import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.widgets import Slider

# ==== Initialize Serial Communication (adjust COM port as needed) ====
try:
    ser = serial.Serial("COM8", 9600, timeout=1)  # Replace COM8 with your Arduino port
    time.sleep(2)  # Wait for serial to connect
except:
    ser = None
    print("⚠️ Could not connect to serial port.")

# ==== Robot Leg Parameters ====
L1 = 100  # Hip to knee
L2 = 156  # Knee to foot

# Angle limits (from LegControl.h)
HIP_BODY_MIN = -43
HIP_BODY_MAX = 37
HIP_LEG_MIN = -247
HIP_LEG_MAX = -43
KNEE_MIN = 0
KNEE_MAX = 150

# Initial angles
theta_knee_init = 90  # Now called theta1
theta_hip_init = -150  # Now called theta2

# ==== Set up Plot ====
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.35)
ax.set_aspect("equal")
ax.set_xlim(-250, 250)
ax.set_ylim(-400, 100)
ax.set_title("2-Joint Robot Leg with Inverse Kinematics (Knee=θ1, Hip=θ2)")

(line,) = ax.plot([], [], "o-", lw=4, markersize=8, color="blue")
(target_dot,) = ax.plot([], [], "rx", markersize=10)


# ==== Draw Robot Arm ====
def update_leg(theta_knee_deg, theta_hip_deg):
    theta1 = np.radians(theta_hip_deg)  # Hip (now θ2)
    theta2 = np.radians(theta_knee_deg)  # Knee (now θ1)

    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    line.set_data([x0, x1, x2], [y0, y1, y2])
    fig.canvas.draw_idle()


# ==== Sliders ====
ax_theta1 = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_theta2 = plt.axes([0.25, 0.1, 0.65, 0.03])
slider_theta1 = Slider(
    ax_theta1, "Knee θ1", KNEE_MIN, KNEE_MAX, valinit=theta_knee_init
)
slider_theta2 = Slider(
    ax_theta2, "Hip θ2", HIP_LEG_MIN, HIP_LEG_MAX, valinit=theta_hip_init
)


# ==== Additional Sliders for X and Y target ====
ax_x = plt.axes([0.25, 0.05, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.0, 0.65, 0.03])
slider_x = Slider(ax_x, "Target X", -200, 200, valinit=0)
slider_y = Slider(ax_y, "Target Y", -400, 100, valinit=0)


# ==== Send angles to Arduino ====
def send_angles(theta_knee, theta_hip):
    if ser:
        try:
            command = f"{int(theta_knee)} {int(theta_hip)} {0}\n"
            ser.write(command.encode())
        except Exception as e:
            print("Serial error:", e)


# ==== On slider change ====
def on_slider_change(val):
    t1 = slider_theta1.val  # Knee
    t2 = slider_theta2.val  # Hip
    update_leg(t1, t2)
    send_angles(t1, t2)


ignore_slider_callback = False


# ==== Update target position ====
def on_xy_change(val):
    global ignore_slider_callback
    if ignore_slider_callback:
        return
    x = slider_x.val
    y = slider_y.val
    apply_ik_and_update(x, y)


slider_theta1.on_changed(on_slider_change)
slider_theta2.on_changed(on_slider_change)

slider_x.on_changed(on_xy_change)
slider_y.on_changed(on_xy_change)


# ==== Inverse Kinematics ====
def inverse_kinematics(x, y):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if np.abs(D) > 1.0:
        return None, None
    theta_knee = np.arccos(D)  # Now θ1
    theta_hip = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(theta_knee), L1 + L2 * np.cos(theta_knee)
    )
    return np.degrees(theta_knee), np.degrees(theta_hip)


# ==== Mouse Interaction ====
dragging = False


def on_press(event):
    global dragging
    if event.inaxes == ax:
        dragging = True


def on_release(event):
    global dragging
    dragging = False


def on_motion(event):
    global dragging
    if dragging and event.inaxes == ax:
        x, y = event.xdata, event.ydata
        apply_ik_and_update(x, y)


def on_click(event):
    if event.inaxes != ax:
        return
    x, y = event.xdata, event.ydata
    apply_ik_and_update(x, y)


def apply_ik_and_update(x, y):
    global ignore_slider_callback

    theta_knee_deg, theta_hip_deg = inverse_kinematics(x, y)
    if theta_knee_deg is None:
        print("Target out of reach.")
        return

    theta_knee_deg = np.clip(theta_knee_deg, KNEE_MIN, KNEE_MAX)
    theta_hip_deg = np.clip(theta_hip_deg, HIP_LEG_MIN, HIP_LEG_MAX)

    ignore_slider_callback = True

    slider_theta1.set_val(theta_knee_deg)
    slider_theta2.set_val(theta_hip_deg)
    slider_x.set_val(x)
    slider_y.set_val(y)

    ignore_slider_callback = False

    target_dot.set_data([x], [y])
    send_angles(theta_knee_deg, theta_hip_deg)
    fig.canvas.draw_idle()


# ==== Register Events ====
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("button_press_event", on_click)

# ==== Initial Draw ====
update_leg(theta_knee_init, theta_hip_init)

plt.show()
