'''
MCMASTER ECE CAPSTONE 2025
UFM (Unidentified Flying Mouse) - Group 10

File Name         : pixart_pnp.py
Originator        : L. West, westl5
Purpose           : Test Code for Localization of User's Wrist in Space Using One PixArt PAJ7025R3
Date Last Modified: 2024/Jan/14 by L. West
'''

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Camera Parameters ---
CONSIDER_DISTORTION = False

# Define camera parameters (from PixArt PAJ7025R3 datasheet)
f_eff = 0.378        # focal length (mm)
pixel_size = 11e-3   # pixel size in mm
resolution = 98      # pixels (sensor is a square)

# Create camera matrix (intrinsic parameters)
fx = fy = f_eff / pixel_size  # focal length in pixels
cx = cy = resolution / 2      # principal point at center
camera_matrix = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
], dtype=np.float32)

#distortion defined as -30% in datasheet. OpenCV uses different model, so this part maty not be accurate
if CONSIDER_DISTORTION:
    # -30% distortion, approximate by setting k1=-0.3 
    dist_coeffs = np.array([[-0.3], [0], [0], [0]], dtype=np.float32)  # k1=-0.3, k2=0, p1=0, p2=0
else:
    dist_coeffs = np.zeros((4,1))
#--- End Camera Parameters ---

#init serial port
ser = serial.Serial(
    port='COM3', #Ryan: COM3, Luke: 
    baudrate=9600,
    timeout=1
)

# Initialize the plot
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-0, 4095)  # Adjust these limits based on your data range
ax.set_ylim(-0, 4095)
ax.grid(True)
ax.set_title('Real-ish time Object Positions')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')

# Initialize scatter plots for each object with different colors
scat0 = ax.scatter([], [], c='red', label='Object 0')
scat1 = ax.scatter([], [], c='blue', label='Object 1')
scat2 = ax.scatter([], [], c='green', label='Object 2')
scat3 = ax.scatter([], [], c='magenta', label='Object 3')
ax.legend()

data_points = {"obj0": (0, 0), "obj1": (0, 0), "obj2": (0, 0), "obj3": (0, 0)}

def update():
    res_b = ser.readline()
    res = res_b.decode()
    res = res.split(',')
    try:
        res = [int(coord.strip()) for coord in res]
        obj0 = (res[0], res[1])
        obj1 = (res[2], res[3])
        obj2 = (res[4], res[5])
        obj3 = (res[6], res[7])
        objs = [obj0, obj1, obj2]
        print(objs)
        return objs
    except:
        pass


def animate(frame):
    data = update()
    if data is not None:
        # Update scatter plot positions
        scat0.set_offsets(data[0])
        scat1.set_offsets(data[1])
        scat2.set_offsets(data[2])
    return scat0, scat1, scat2

# Create animation
anim = FuncAnimation(fig, animate, interval=1, blit=True)

# Show the plot
plt.show()