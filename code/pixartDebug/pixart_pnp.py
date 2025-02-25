'''
MCMASTER ECE CAPSTONE 2025
UFM (Unidentified Flying Mouse) - Group 10

File Name         : pixart_pnp.py
Originator        : L. West, westl5
Purpose           : Test Code for Mouse Cursor Movement Using PixArt PAJ7025R3
Date Last Modified: 2024/Jan/14 by L. West
'''

import serial
#import keyboard as kb
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import move_mouse as mm 
import time

offsetx = 4095/2
offsety = 4095/2

CURSOR_LIMIT_PERCENT = 200  # Cursor movement limit as percentage of pattern size (To be changed in the future, once occlusion and/or missing points are considered)

#--- LED Object Parameters ---
# Define object colors in BGR format
obj_colors = [
    (0, 0, 255),  # red
    (255, 0, 0),  # blue
    (0, 255, 0),  # green
    (255, 0, 255)  # magenta
]
#--- End Object Parameters ---

# --- Camera Parameters ---
CONSIDER_DISTORTION = False

# Define camera parameters (from PixArt PAJ7025R3 datasheet)
f_eff = 0.378e-3        # focal length (mm)
pixel_size = 11e-6   # pixel size in mm
resolution = 4095 # pixels (sensor is a square)

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
    dist_coeffs = np.array([[0.3], [0], [0], [0]], dtype=np.float32)  # k1=-0.3, k2=0, p1=0, p2=0
else:
    dist_coeffs = np.zeros((4,1))
#--- End Camera Parameters ---

#init serial port
ser = serial.Serial(
    port='/dev/cu.usbserial-0001', #/dev/cu.usbxsserial-0001', #Ryan: COM3, Luke: 
    baudrate=9600,
    timeout=1
)

# Initialize the plot
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(0, 4095)  # Adjust these limits based on your data range
ax.set_ylim(0, 4095)
ax.grid(True)
ax.set_title('Real-ish time Object Positions')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')

# Initialize scatter plots for each object with different colors
scat0 = ax.scatter([], [], c='red', label='Object 0')
scat1 = ax.scatter([], [], c='blue', label='Object 1')
scat2 = ax.scatter([], [], c='green', label='Object 2')
scat3 = ax.scatter([], [], c='magenta', label='Object 3')
scat_cursor = ax.scatter([], [], c='black', label='Cursor')
ax.legend()


#get numpy array of 2d points from objs in order 
def get_points(objs):
    '''
    returns 4 np.array, each containing a coordinate, all in a one big np.array
    Also implicitly converts each value to float?...
    '''
    points = []
    try:
        for i in range(len(objs)):
            points.append(np.array([objs[i][0], objs[i][1]], dtype=np.float32))
        return np.array(points)
    except Exception as e:
        print(e)
        return np.array(points)

def create_rect_pattern():
    '''
    Creates a rect pattern with 4 points
    '''
    rect_width = 70.5e-3 #LED object irl width
    rect_height = 42.5e-3 #LED object irl height

    # rect points (z=0)
    points = np.array([
        [-rect_width/2, rect_height/2, 0],    # Top left
        [rect_width/2, rect_height/2, 0],    # Top right
        [-rect_width/2, -rect_height/2, 0],  # Bottom left
        [rect_width/2, -rect_height/2, 0],   # Bottom right
        
    ], dtype=np.float32)

    return points

#camera cannot be below points, recovered z param is +ve
def ensure_positive_z(R, t):
    '''Ensure camera is on the positive Z side of the points'''
    pos = -R.T @ t
    if pos[2] < 0:  # If camera is on negative Z side
        print("Flipping camera to positive Z side...")
        # Rotate 180 degrees around X axis to flip Z
        R_flip = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        R = R_flip @ R
        t = R_flip @ t
    return R, t

# Convert camera position to screen coordinates
def camera_to_screen(camera_pos, screen_width, screen_height):
    """
    Convert camera position to screen coordinates
    Maps pattern_spacing to screen edges
    +ve Y moves cursor up, +ve X moves cursor to the right
    """
    # Define mapping range using percentage of pattern spacing (temporary fix for occlusion)
    limit = CURSOR_LIMIT_PERCENT  # Convert percentage to decimal
    #x_range = [-limit, limit]  # mm
   # y_range = [-limit, limit]  # mm
    
    # Clip camera position to range
    x = camera_pos[0]
    y = camera_pos[1]
    
    # Map to screen coordinates
    screen_x = x * screen_width
    screen_y = y * screen_height
    
    # Add text to indicate if position is clipped
    clipped = False
    if abs(camera_pos[0]) > limit or abs(camera_pos[1]) > limit:
        clipped = True
    
    return screen_x, screen_y, clipped

# Get the 3D position of the pixart relative to the object
def get_cam_pose(points_pixart_2d, points_obj_3d, camera_matrix, dist_coeffs):

    # Reshape to Nx2 array
    points_pixart_2d = points_pixart_2d.reshape(-1, 2)
    points_obj_3d = points_obj_3d.reshape(-1, 3)

    # Try different methods to see multiple solutions
    print("\nTesting different PnP methods and solutions:")
    '''
    # P3P (up to 4 solutions)
    try:
        retval, rvecs, tvecs = cv.solveP3P(
            points_obj_3d[:3],  # Use first 3 points
            points_pixart_2d[:3], 
            camera_matrix, 
            dist_coeffs,
            flags=cv.SOLVEPNP_P3P
        )
        print(f"\nP3P solutions found: {len(rvecs)}")
        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            R, _ = cv.Rodrigues(rvec)
            R, tvec = ensure_positive_z(R, tvec)
            pos = -R.T @ tvec
            print(f"Solution {i+1}:")
            print(f"  Position: {pos.flatten()}")
    except cv.error as e:
        print("P3P failed:", e)

    # EPNP (usually 1 solution)
    try:
        ret, rvec, tvec = cv.solvePnP(
            points_obj_3d, 
            points_pixart_2d, 
            camera_matrix, 
            dist_coeffs,
            flags=cv.SOLVEPNP_EPNP
        )
        R, _ = cv.Rodrigues(rvec)
        R, tvec = ensure_positive_z(R, tvec)
        pos = -R.T @ tvec
        print(f"\nEPnP solution:")
        print(f"  Position: {pos.flatten()}")
    except cv.error as e:
        print("EPnP failed:", e)
    '''
    # IPPE (multiple solutions)
    try:
        ret, rvec, tvec = cv.solvePnP(
            points_obj_3d, 
            points_pixart_2d, 
            camera_matrix, 
            dist_coeffs,
            flags=cv.SOLVEPNP_EPNP
        )
        R, _ = cv.Rodrigues(rvec)
        R, tvec = ensure_positive_z(R, tvec)
        pos = -R.T @ tvec
        print(f"\nIPPE solution:")
        print(f"  Position: {pos.flatten()}")
    except cv.error as e:
        print("IPPE failed:", e)

    '''
    # ITERATIVE (refines to single solution)
    try:
        ret, rvec, tvec = cv.solvePnP(
            points_obj_3d, 
            points_pixart_2d, 
            camera_matrix, 
            dist_coeffs,
            flags=cv.SOLVEPNP_ITERATIVE
        )
        R, _ = cv.Rodrigues(rvec)
        R, tvec = ensure_positive_z(R, tvec)
        pos = -R.T @ tvec
        print(f"\nIterative solution:")
        print(f"  Position: {pos.flatten()}")
    except cv.error as e:
        print("Iterative method failed:", e)
    '''
    # Get initial recovered camera position
    recovered_pos = -R.T @ tvec

    # Return the 3D position of the object
    return recovered_pos


def wii_method(points_2d):
    """
    Implements Wii-style cursor movement using IR dot positions
    Args:
        points_2d: List of (x,y) coordinates from IR sensor
    Returns:
        cursor_x, cursor_y: Normalized cursor position (-1 to 1)
    """
    try:
        # Convert points to numpy array if not already
        points = np.array(points_2d)
        
        # Calculate center point between all dots
        center = np.mean(points, axis=0)
        
        # Calculate distance between furthest points (pattern width)
        distances = []
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                dist = np.linalg.norm(points[i] - points[j])
                distances.append(dist)
        pattern_width = max(distances)
        
        # Normalize center position by pattern size
        # Smaller pattern_width = further away = larger cursor movement
        #scale_factor = (1 + pattern_width / resolution) if pattern_width > 0 else 1.0
        
        # Convert sensor coordinates to cursor coordinates
        # Sensor coordinates: (0,0) at top-left
        # Cursor coordinates: (-1,-1) at bottom-left, (1,1) at top-right
        cursor_x = (center[0] - resolution/2) * 2 / resolution
        cursor_y = (center[1] - resolution/2)*2 /resolution # Invert Y axis
        return cursor_x, cursor_y
        
    except Exception as e:
        print(f"Wii method calculation failed: {e}")
        return 0, 0

def update() -> list[tuple[int, int]]:
    '''
    returns 4 coordinate tuples in a list
    '''
    data = b''
    while data != b'\n':
        data = ser.read()
    data = b''
    while data[-1:] != b'\n':
        data += ser.read()
    ser.reset_input_buffer()
    data = data.decode('utf-8').strip()
    res = data.split(',')
    try:
        res = [int(coord.strip()) for coord in res]
        obj0 = (res[0], res[1])
        obj1 = (res[2], res[3])
        obj2 = (res[4], res[5])
        obj3 = (res[6], res[7])
        objs = [obj0, obj1, obj2, obj3]
        return objs
    except:
        pass

"""
def animate(frame):
    data = update()
    points_pixart_2d = get_points(data)
    # Get camera pose
    camera_pos = get_cam_pose(points_pixart_2d, points_obj_3d, camera_matrix, dist_coeffs)
    print("Camera position:", camera_pos)

    #get cursor position
    screen_width = 4095 # screen width in pixels
    screen_height = 4095  # screen height in pixels

    cursor_x, cursor_y, clipped = camera_to_screen(camera_pos, screen_width, screen_height)
    print("Cursor position:", cursor_x, cursor_y)
    print("Clipped:", clipped)

    cursor = [float(cursor_x)+offsetx, float(cursor_y)+offsety]

    if data is not None:
        # Update scatter plot positions
        scat0.set_offsets(data[0])
        scat1.set_offsets(data[1])
        scat2.set_offsets(data[2])
        scat3.set_offsets(data[3])
        scat_cursor.set_offsets(cursor)

    return scat0, scat1, scat2, scat3, scat_cursor
"""

# Update the animate function to use wii_method instead of PnP
def animate(frame):
    data = update()
    if data is not None:
        points_2d = get_points(data)
        
        # Get cursor position using Wii method
        cursor_x, cursor_y = wii_method(points_2d)
        
        # Scale to screen coordinates
        screen_y = (cursor_x + 1) * 4095/2 
        screen_x = (cursor_y + 1) * 4095/2 
        
        cursor = [float(screen_x), float(screen_y)]
        
        # Update scatter plot positions
        scat0.set_offsets(data[0])
        scat1.set_offsets(data[1])
        scat2.set_offsets(data[2])
        scat3.set_offsets(data[3])
        scat_cursor.set_offsets(cursor)
        
    return scat0, scat1, scat2, scat3, scat_cursor

# Create pattern points
points_obj_3d = create_rect_pattern()

# Create animation
#anim = FuncAnimation(fig, animate, interval=1, blit=True)

# Show the plot
#plt.show()


# Example usage
def main():
    # Create controller
    mouse = mm.MouseController()
    while True:
        
        data = update()
        if data is not None:

            points_2d = get_points(data)
                
            # Get cursor position using Wii method
            cursor_x, cursor_y = wii_method(points_2d)
                
            # Scale to screen coordinates
            screen_x = (cursor_x) * 1920 
            screen_y = (cursor_y) * 1080 
            print(f"Cursor position: ({screen_x}, {screen_y})")
            mouse.move_to(screen_x, screen_y)
        
        # Get current position
        pos = mouse.get_position()
        print(f"Final position: ({pos.x}, {pos.y})")

if __name__ == "__main__":
    main()