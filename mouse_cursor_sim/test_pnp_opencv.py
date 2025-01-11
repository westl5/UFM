'''
MCMASTER ECE CAPSTONE 2025
UFM (Unidentified Flying Mouse) - Group 10

File Name         : test_pnp_opencv.py
Originator        : L. West, westl5
Purpose           : Test Code for Localization of User's Wrist in Space Using One PixArt PAJ7025R3
Date Last Modified: 2024/Jan/03 by L. West
'''

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# Configuration flags
PLOT_VIEW_DIR = True # Set to True to show viewing directions and looking points
CONSIDER_DISTORTION = True
CONSIDER_NOISE = True

PATTERN_TYPE = 'six'  # Options: 'square' or 'six' or 'test_shape'

BASE_DISTANCE = 100  # Distance between base LEDs in mm for 6-point pattern
CURSOR_LIMIT_PERCENT = 200  # Cursor movement limit as percentage of pattern size (To be changed in the future, once occlusion and/or missing points are considered)
NOISE_STDEV = 0.2  # Standard deviation of Gaussian noise added to 2D points

# Define camera parameters (from PixArt PAJ7025R3 datasheet)
f_eff = 0.378        # focal length (mm)
pixel_size = 11e-3   # mm
resolution = 98      # pixels (square sensor)

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

test_num = -70
# Define dictionary of camera positions and their targets
CAMERA_POSITIONS = {
    'front': (np.array([0, 0, 200]), np.array([0, 0, 0])),      # Looking straight down
    'side': (np.array([-100, 0, 100]), np.array([0, 0, 0])),     # Side view from high angle
    'angle': (np.array([-100, -100, 200]), np.array([0, 0, 0])),   # 45-degree angle view
    'high': (np.array([-25, -25, 150]), np.array([0, 0, 0])),   # High overhead view
    'close': (np.array([-10, -10, 50]), np.array([0, 0, 0])),   # Close overhead view
    'far': (np.array([test_num, 50, 200]), np.array([test_num, 50, 0])),    # Far overhead view
    'offset': (np.array([-50, 25, 200]), np.array([-50, 25, 0]))   # Offset overhead view
}

# Select camera position (change this string to test different positions)
position_type = 'angle'  # Options: 'front', 'side', 'angle', 'high', 'close', 'far', 'offset'

# Get camera position and target
camera_pos, target_point = CAMERA_POSITIONS[position_type]

print(f"\nUsing camera position: {position_type}")
print(f"Camera position: {camera_pos}")
print(f"Target point: {target_point}")

#--------- HELPER FUNCTIONS ------------

def create_square_pattern():
    '''
    Creates a square pattern with 4 points
    100mm separation between points
    '''
    spacing =  BASE_DISTANCE # Distance between points (100mm)
    
    # Square points (z=0)
    points = np.array([
        [-spacing/2, -spacing/2, 0],  # Bottom left
        [spacing/2, -spacing/2, 0],   # Bottom right
        [spacing/2, spacing/2, 0],    # Top right
        [-spacing/2, spacing/2, 0]    # Top left
    ], dtype=np.float32)

    dimensions = {
        'spacing': spacing,
        'num_points': 4,
        'type': 'square'
    }
    
    return points, dimensions

def create_test_pattern():
    '''
    Creates a test patttern with any number of points
    Will be modified and used to test the PnP solver
    -Try different number of points
    -Try different point spacing
    -Try different base shapes
    All dimenrions in mm
    '''
    spacing = BASE_DISTANCE  # Use configurable base distance

    points = np.array([
        [-spacing/2, -spacing/2, 0],    # Bottom left
        [spacing/2, -spacing/2, 0],     # Bottom right
        [spacing/2, spacing/2, 0],      # Top right
        [-spacing/2, spacing/2, 0],      # Top left
        [-spacing/4, -spacing/4, 0],
        [spacing/4, -spacing/4, 0],
        [spacing/4, spacing/3, 0],
                                                  # center point at 50mm height
    ], dtype=np.float32)

    dimensions = {
        'spacing': spacing,
        'heights': [points[4][2]], 
        'num_points': 7,
        'type': 'test_shape'
    }

    return points, dimensions

def create_six_point_pattern():
    '''
    Creates a 3D point pattern with 6 IR LED points:
    - Base triangle (3 points) with configurable spacing
    - Three points at practical heights forming a second triangle
    All dimensions in mm
    '''
    spacing = BASE_DISTANCE  # Use configurable base distance
    
    # Base triangle points (z=0)
    base_points = [
        [-spacing/2, -spacing/2, 0],     # Bottom left
        [spacing/2, -spacing/2, 0],      # Bottom right
        [0, spacing/2, 0],               # Top center
    ]
    
    # Upper points at practical heights
    upper_points = [
        [-spacing/4, -spacing/4, 10],     # Left height (5mm)
        [spacing/4, -spacing/4, 20],     # Right height (10mm)
        [0, spacing/4, 30],               # Center height (15mm)
    ]
    
    points = np.array(base_points + upper_points, dtype=np.float32)
    
    dimensions = {
        'spacing': spacing,
        'heights': [[15], [30], [45]], # Practical heights for building
        'num_points': 6,
        'type': 'six'
    }
    
    return points, dimensions


def plot_pattern_3d(ax, points, pattern_type):
    """Plot the calibration pattern based on its type"""
    # Plot all points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
              c='r', marker='o', s=100, label='Pattern Points')
    
    if pattern_type == 'square':
        # Draw square
        edges = np.array([0, 1, 2, 3, 0])
        ax.plot(points[edges, 0], points[edges, 1], points[edges, 2], 
               'r--', alpha=0.7)
    elif pattern_type == 'test_shape':
        # Draw test_shape
        base_edges = np.array([0, 1, 2, 3, 0])
        ax.plot(points[base_edges, 0], points[base_edges, 1], points[base_edges, 2], 
               'r--', alpha=0.7)
        #draw lines to top point from each base point
        upper_edges = np.array([0, 4, 1])
        ax.plot(points[upper_edges, 0], points[upper_edges, 1], points[upper_edges, 2], 
               'r--', alpha=0.7)
        upper_edges = np.array([2, 4, 3])
        ax.plot(points[upper_edges, 0], points[upper_edges, 1], points[upper_edges, 2], 
               'r--', alpha=0.7)
        
    elif pattern_type == 'six':  # 'six' pattern
        # Draw base triangle
        base_edges = np.array([0, 1, 2, 0])
        ax.plot(points[base_edges, 0], points[base_edges, 1], points[base_edges, 2], 
               'r--', alpha=0.7)
        
        upper_edges = np.array([3, 4, 5, 3])
        ax.plot(points[upper_edges, 0], points[upper_edges, 1], points[upper_edges, 2], 
               'r--', alpha=0.7)
        
        for i in range(3):
            ax.plot([points[i][0], points[i+3][0]], 
                   [points[i][1], points[i+3][1]], 
                   [points[i][2], points[i+3][2]], 
                   'r-', alpha=0.7)
    else:
        raise ValueError(f"Invalid pattern type: {pattern_type}")
    
def addNoise(points, mean, std_dev, shape):
    '''
    Adds gaussian noise to the points
    '''
    gaussian_noise = np.random.normal(mean, std_dev, shape)
    return points + gaussian_noise

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

#!!! to be changed to work more like the pixart camera, which tracks the points !!!
# Add after projecting the points but before plotting:

def get_display_order(points_2d):
    '''
    Returns indices that would sort points from top to bottom, left to right
    in the 2D projection (matches how pixart tracks points)
    '''
    # Create array of (y, x) coordinates for sorting
    yx_coords = np.column_stack((-points_2d[:, 1], points_2d[:, 0]))  # Negative y for top-to-bottom
    
    # Get sorting indices based on y first, then x
    return np.lexsort((yx_coords[:, 1], yx_coords[:, 0]))

# Convert camera position to screen coordinates
def camera_to_screen(camera_pos, screen_width, screen_height, pattern_spacing):
    """
    Convert camera position to screen coordinates
    Maps pattern_spacing to screen edges
    +ve Y moves cursor up, +ve X moves cursor to the right
    """
    # Define mapping range using percentage of pattern spacing (temporary fix for occlusion)
    limit = pattern_spacing * (CURSOR_LIMIT_PERCENT/100)  # Convert percentage to decimal
    x_range = [-limit, limit]  # mm
    y_range = [-limit, limit]  # mm
    
    # Clip camera position to range
    x = np.clip(camera_pos[0], x_range[0], x_range[1])
    y = np.clip(camera_pos[1], y_range[0], y_range[1])
    
    # Map to screen coordinates
    screen_x = ((x - x_range[0]) / (x_range[1] - x_range[0])) * screen_width
    screen_y = ((y - y_range[0]) / (y_range[1] - y_range[0])) * screen_height
    
    # Add text to indicate if position is clipped
    clipped = False
    if abs(camera_pos[0]) > limit or abs(camera_pos[1]) > limit:
        clipped = True
    
    return screen_x, screen_y, clipped

#--------- MAIN CODE ------------
    
if PATTERN_TYPE == 'square':
    points_3d, pattern_dims = create_square_pattern()
elif PATTERN_TYPE == 'six':
    points_3d, pattern_dims = create_six_point_pattern()
elif PATTERN_TYPE == 'test_shape':
    points_3d, pattern_dims = create_test_pattern()
else:
    raise ValueError(f"Invalid pattern type: {PATTERN_TYPE}")

# Update pattern information printing
print("LED Object Information:")
print(f"Object type: {pattern_dims['type']}")
print(f"Number of points: {pattern_dims['num_points']}")
print(f"Point spacing: {pattern_dims['spacing']} mm")
if pattern_dims['type'] == 'six' or pattern_dims['type'] == 'test_shape':
    print(f"Upper point heights: {pattern_dims['heights']} mm")
print("\nPoint coordinates:")
for i, point in enumerate(points_3d):
    print(f"Point {i+1}: {point}")

# Get forward unit vector for true camera 
forward = target_point - camera_pos
forward = forward / np.linalg.norm(forward)

# Calculate right vector, handling the case when forward is parallel to up
right = np.cross(forward, np.array([0, 0, 1]))
if np.allclose(right, 0):  # If forward is parallel to up
    right = np.cross(forward, np.array([1, 0, 0]))  # Use x-axis instead
right = right / np.linalg.norm(right)

# Calculate up vector
up = np.cross(right, forward)
up = up / np.linalg.norm(up)

# Create rotation matrix from camera vectors
R_true = np.vstack([-right, -up, -forward])
t_true = -R_true @ camera_pos

# Convert to rotation vector
rvec_true, _ = cv.Rodrigues(R_true)

# Project points using true camera pose
points_2d_cv, _ = cv.projectPoints(points_3d, rvec_true, t_true, camera_matrix, dist_coeffs)
points_2d_cv = points_2d_cv.reshape(-1, 2)

if CONSIDER_NOISE:
    # Add Gaussian noise to 2D points
    points_2d_cv = addNoise(points_2d_cv, 0, NOISE_STDEV, points_2d_cv.shape)

# Try different methods to see multiple solutions
print("\nTesting different PnP methods and solutions:")

# P3P (up to 4 solutions)
try:
    retval, rvecs, tvecs = cv.solveP3P(
        points_3d[:3],  # Use first 3 points
        points_2d_cv[:3], 
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
        print(f"  Distance from true: {np.linalg.norm(camera_pos - pos.flatten()):.3f} mm")
except cv.error as e:
    print("P3P failed:", e)

# EPNP (usually 1 solution)
try:
    ret, rvec, tvec = cv.solvePnP(
        points_3d, 
        points_2d_cv, 
        camera_matrix, 
        dist_coeffs,
        flags=cv.SOLVEPNP_EPNP
    )
    R, _ = cv.Rodrigues(rvec)
    R, tvec = ensure_positive_z(R, tvec)
    pos = -R.T @ tvec
    print(f"\nEPnP solution:")
    print(f"  Position: {pos.flatten()}")
    print(f"  Distance from true: {np.linalg.norm(camera_pos - pos.flatten()):.3f} mm")
except cv.error as e:
    print("EPnP failed:", e)

# ITERATIVE (refines to single solution)
try:
    ret, rvec, tvec = cv.solvePnP(
        points_3d, 
        points_2d_cv, 
        camera_matrix, 
        dist_coeffs,
        flags=cv.SOLVEPNP_ITERATIVE
    )
    R, _ = cv.Rodrigues(rvec)
    R, tvec = ensure_positive_z(R, tvec)
    pos = -R.T @ tvec
    print(f"\nIterative solution:")
    print(f"  Position: {pos.flatten()}")
    print(f"  Distance from true: {np.linalg.norm(camera_pos - pos.flatten()):.3f} mm")
except cv.error as e:
    print("Iterative method failed:", e)

# Convert rotation vector to matrix
R, _ = cv.Rodrigues(rvec)

# Get initial recovered camera position
recovered_pos = -R.T @ tvec

# Calculate looking directions properly
true_forward = (target_point - camera_pos) / np.linalg.norm(target_point - camera_pos)
recovered_forward = -R[:, 2]  # The camera looks along negative Z in camera coordinates

# Calculate where each camera is looking
true_looking_point = target_point
recovered_looking_point = recovered_pos + recovered_forward * np.linalg.norm(target_point - camera_pos)

# Print verification
print("\nCamera Direction Verification:")
print("True camera:")
print(f"  Position: {camera_pos}")
print(f"  Looking direction: {true_forward}")
print(f"  Looking at point: {true_looking_point}")

print("\nRecovered camera:")
print(f"  Position: {recovered_pos.flatten()}")
print(f"  Looking direction: {recovered_forward}")
print(f"  Looking at point: {recovered_looking_point}")

print("\nDirection Analysis:")
print(f"Direction dot product: {np.dot(true_forward, recovered_forward):.3f}")
print(f"Direction angle: {np.arccos(np.clip(np.dot(true_forward, recovered_forward), -1.0, 1.0))*180/np.pi:.1f} degrees")

# Plot results
fig = plt.figure(figsize=(15, 12))  # accomodates for 3 plots
gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])  # 2 rows, 2 columns grid

# 3D subplot (top left)
ax_3d = fig.add_subplot(gs[0, 0], projection='3d')

# Plot 3D points
ax_3d.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], 
             c='r', marker='o', s=100, label='Grid Points')

# Plot camera positions
ax_3d.scatter(camera_pos[0], camera_pos[1], camera_pos[2], 
             c='b', marker='^', s=200, label='True Camera')
ax_3d.scatter(recovered_pos[0], recovered_pos[1], recovered_pos[2], 
             c='g', marker='^', s=200, label='Recovered Camera')

# Draw viewing directions and looking points
if PLOT_VIEW_DIR:
    direction_scale = 20

    # True camera
    ax_3d.quiver(camera_pos[0], camera_pos[1], camera_pos[2],
                true_forward[0], true_forward[1], true_forward[2],
                color='b', length=direction_scale, alpha=0.5)
    ax_3d.scatter(true_looking_point[0], true_looking_point[1], true_looking_point[2],
                 c='b', marker='x', s=100, label='True Looking Point')

    # Recovered camera
    ax_3d.quiver(recovered_pos[0], recovered_pos[1], recovered_pos[2],
                recovered_forward[0], recovered_forward[1], recovered_forward[2],
                color='g', length=direction_scale, alpha=0.5)
    ax_3d.scatter(recovered_looking_point[0], recovered_looking_point[1], recovered_looking_point[2],
                 c='g', marker='x', s=100, label='Recovered Looking Point')

ax_3d.set_title('3D View')
ax_3d.set_xlabel('X (mm)')
ax_3d.set_ylabel('Y (mm)')
ax_3d.set_zlabel('Z (mm)')
ax_3d.legend()
ax_3d.set_box_aspect([1,1,1])
ax_3d.view_init(elev=30, azim=45)

# Get display order based on projected points
display_order = get_display_order(points_2d_cv)

# 3D subplot - add point numbers
for i, idx in enumerate(display_order):
    point = points_3d[idx]
    ax_3d.text(point[0], point[1], point[2], f' {i+1}', 
               color='red', fontsize=10)

# 2D subplot (top right)
ax_2d = fig.add_subplot(gs[0,1])

# Plot original projections
ax_2d.scatter(points_2d_cv[:, 0], points_2d_cv[:, 1], 
             c='r', marker='o', label='True Projections')

# Project points using recovered pose
projected_points_pnp, _ = cv.projectPoints(points_3d, rvec, tvec, camera_matrix, dist_coeffs)
projected_points_pnp = projected_points_pnp.reshape(-1, 2)

# Plot PnP projections
ax_2d.scatter(projected_points_pnp[:, 0], projected_points_pnp[:, 1], 
             c='g', marker='x', label='Recovered Projections')

# Set axis properties
ax_2d.set_xlim([0, resolution])
ax_2d.set_ylim([resolution, 0])
ax_2d.grid(True)
ax_2d.set_title('2D Projections')
ax_2d.set_xlabel('Pixel X')
ax_2d.set_ylabel('Pixel Y')
ax_2d.legend()
ax_2d.set_aspect('equal')

# 2D subplot - add point numbers
for i, idx in enumerate(display_order):
    point = points_2d_cv[idx]
    ax_2d.text(point[0], point[1], f' {i+1}', 
               color='orange', fontsize=10)
    # Also add numbers to PnP projections
    pnp_point = projected_points_pnp[idx]
    ax_2d.text(pnp_point[0], pnp_point[1], f' {i+1}', 
               color='blue', fontsize=10)

# Update point coordinate printing
print("\nPoint coordinates:")
for i, idx in enumerate(display_order):
    print(f"Point {i+1}: {points_3d[idx]}")

# Screen subplot (bottom, spans both columns)
ax_screen = fig.add_subplot(gs[1, :])

# Screen dimensions
screen_width = 1920
screen_height = 1080
screen_aspect = screen_width / screen_height

# Get cursor positions for both true and recovered cameras
true_cursor_x, true_cursor_y, true_clipped = camera_to_screen(
    camera_pos, screen_width, screen_height, pattern_dims['spacing']
)
recovered_cursor_x, recovered_cursor_y, recovered_clipped = camera_to_screen(
    recovered_pos.flatten(), screen_width, screen_height, pattern_dims['spacing']
)

# Plot screen
ax_screen.set_xlim([0, screen_width])
ax_screen.set_ylim([0, screen_height])
ax_screen.set_aspect('equal')

# Draw screen border
ax_screen.plot([0, screen_width, screen_width, 0, 0], 
               [0, 0, screen_height, screen_height, 0], 'k-')

# Plot cursors
ax_screen.plot(true_cursor_x, true_cursor_y, 'bo', markersize=10, label='True Cursor')
ax_screen.plot(recovered_cursor_x, recovered_cursor_y, 'go', markersize=10, label='Recovered Cursor')

# Add cursor position text with clipping indication
clip_text = ""
if true_clipped:
    clip_text = " (CLIPPED)"
ax_screen.text(50, 50, f'True Cursor: ({int(true_cursor_x)}, {int(true_cursor_y)}){clip_text}', 
               color='blue')

clip_text = ""
if recovered_clipped:
    clip_text = " (CLIPPED)"
ax_screen.text(50, 100, f'Recovered Cursor: ({int(recovered_cursor_x)}, {int(recovered_cursor_y)}){clip_text}', 
               color='green')

ax_screen.set_title('Screen Cursor Position')
ax_screen.set_xlabel('Screen Width (pixels)')
ax_screen.set_ylabel('Screen Height (pixels)')
ax_screen.legend()
ax_screen.grid(True)

plt.tight_layout()
plt.show()

print("true rotation:")
print(rvec_true)
print("true translation:")
print(t_true)

print("recovered rotation:")
print(rvec)
print("recovered translation:")
print(tvec.flatten())


