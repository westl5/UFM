import serial
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import serial
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class HornMethod:
    def __init__(self, world_points, camera_points):
        """
        Initialize with corresponding 3D world points and 2D camera points
        world_points: Nx3 array of 3D coordinates in world frame
        camera_points: Nx2 array of 2D coordinates in camera frame
        """
        self.world_points = world_points
        # Convert 2D points to 3D rays in camera frame
        self.camera_points = np.hstack((camera_points, np.ones((len(camera_points), 1))))
        self.camera_points = self.camera_points / np.linalg.norm(self.camera_points, axis=1)[:, np.newaxis]

    def find_rotation_translation(self):
        """Implement Horn's method to find rotation and translation"""
        # Calculate centroids
        world_centroid = np.mean(self.world_points, axis=0)
        camera_centroid = np.mean(self.camera_points, axis=0)

        # Center the points
        world_centered = self.world_points - world_centroid
        camera_centered = self.camera_points - camera_centroid

        # Calculate covariance matrix
        H = camera_centered.T @ world_centered

        # SVD decomposition
        U, _, Vt = np.linalg.svd(H)
        
        # Ensure right-handed coordinate system
        d = np.linalg.det(U @ Vt)
        S = np.eye(3)
        if d < 0:
            S[2,2] = -1
            
        # Calculate rotation matrix
        R = U @ S @ Vt

        # Calculate translation
        t = world_centroid - R @ camera_centroid

        return R, t

    def get_camera_position(self):
        """Return camera position in world coordinates"""
        R, t = self.find_rotation_translation()
        # Camera position is -R^T * t
        camera_pos = -R.T @ t
        return camera_pos, R

def estimate_camera_pose(world_points, camera_points):
    """
    Estimate camera pose using Horn's method
    Returns camera position and rotation matrix
    """
    horn = HornMethod(world_points, camera_points)
    return horn.get_camera_position()

# Example usage:
if __name__ == "__main__":
    # Define known 3D world points (e.g., LED positions)
    # Define
    world_points = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0]
    ])

    # Get 2D points from camera (example values)
    camera_points = np.array([
        [100, 100],
        [200, 100],
        [200, 200],
        [100, 200]
    ])

    # Estimate camera pose
    camera_position, rotation_matrix = estimate_camera_pose(world_points, camera_points)
    print("Camera position:", camera_position)
    print("Rotation matrix:", rotation_matrix)