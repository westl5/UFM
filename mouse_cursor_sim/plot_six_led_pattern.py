import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_six_point_pattern():
    """
    Creates a 3D calibration pattern with 6 IR LED points:
    - Base triangle (3 points) with configurable spacing
    - Three points at practical heights forming a second triangle
    All dimensions in mm
    """
    spacing = 200  # Base distance between LEDs in mm
    
    # Base triangle points (z=0)
    base_points = [
        [-spacing/2, -spacing/2, 0],     # Bottom left (P1)
        [spacing/2, -spacing/2, 0],      # Bottom right (P2)
        [0, spacing/2, 0],               # Top center (P3)
    ]
    
    # Upper points at practical heights
    upper_points = [
        [-spacing/4, -spacing/4, 5],     # Left post (P4, 5 mm)
        [spacing/4, -spacing/4, 10],     # Right post (P5, 10 mm)
        [0, spacing/4, 15],               # Center post (P6, 15 mm)
    ]
    
    points = np.array(base_points + upper_points, dtype=np.float32)
    return points

def plot_six_led_pattern(points):
    """Plot the 3D configuration of the six LED points and show coordinates."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot 3D points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
               c='r', marker='o', s=100, label='LED Points')

    # Draw base triangle with blue dotted line
    base_edges = np.array([0, 1, 2, 0])
    ax.plot(points[base_edges, 0], points[base_edges, 1], points[base_edges, 2], 
            'b:', alpha=0.7, label='Base Triangle')  # Blue dotted line

    # Draw upper triangle with red dotted line
    upper_edges = np.array([3, 4, 5, 3])  # Connect upper points
    ax.plot(points[upper_edges, 0], points[upper_edges, 1], points[upper_edges, 2], 
            'm:', alpha=0.7, label='Upper Triangle')  # Magenta dotted line

    # Draw line between P3 and P6 with cyan dotted line
    ax.plot([points[2][0], points[5][0]], 
            [points[2][1], points[5][1]], 
            [points[2][2], points[5][2]], 
            'c:', alpha=0.7, linewidth=2)  # Cyan dotted line

    # Draw lines between P5 and P2, and P1 and P4 with cyan dotted lines
    ax.plot([points[1][0], points[4][0]], 
            [points[1][1], points[4][1]], 
            [points[1][2], points[4][2]], 
            'c:', alpha=0.7, linewidth=2)  # Cyan dotted line

    ax.plot([points[0][0], points[3][0]], 
            [points[0][1], points[3][1]], 
            [points[0][2], points[3][2]], 
            'c:', alpha=0.7, linewidth=2)  # Cyan dotted line

    # Set labels and title
    ax.set_title('3D Configuration of Six LED Points')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.legend()

    # Set limits for better visualization
    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])
    ax.set_zlim([0, 50])

    # Annotate coordinates of each point
    for i, point in enumerate(points):
        ax.text(point[0], point[1], point[2], f'P{i+1}: ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})', color='black')

    plt.show()

if __name__ == "__main__":
    points = create_six_point_pattern()
    plot_six_led_pattern(points)