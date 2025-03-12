import numpy as np
import serial
import pyautogui
import time
from scipy.spatial import distance
pyautogui.FAILSAFE = False

class IRCursorController:
    def __init__(self, sensor_interface):
        # Sensor interface that provides point coordinates
        self.sensor = sensor_interface
        
        # Sensitivity of mouse movement (adjust as needed)
        self.sensitivity = 2.0
        
        # Previous centroid position
        self.prev_centroid = None
        
        # Previous identified points (for tracking consistency)
        self.prev_identified_points = None
        
        # Rectangle pattern dimensions 
        self.rectangle_width = 70e-3  # Long side
        self.rectangle_height = 40e-3 # Short side
        
        # Expected rectangular pattern (relative coordinates)
        self.reference_pattern = np.array([
            [0, 0],                          # Top-left
            [self.rectangle_width, 0],       # Top-right
            [0, self.rectangle_height],      # Bottom-left
            [self.rectangle_width, self.rectangle_height]  # Bottom-right
        ], dtype=np.float32)
        
        # Pixart PAJ7025R3 intrinsic parameters (from datasheet)
        focal_length_mm = 0.378e-3  # 0.378mm
        sensor_width_mm = 11e-6  # 11um
        pixel_resolution = 98  # 98x98 pixels
        fx = fy = focal_length_mm / sensor_width_mm
        cx = cy = pixel_resolution / 2
        self.focal_length = (fx, fy)  # fx, fy
        self.principal_point = (cx, cy)  # cx, cy
        
        # Flag to check if system is calibrated
        self.is_calibrated = False
        
        # Threshold for minimum brightness to consider a point valid
        self.min_brightness = 50
        
        # Threshold for minimum area to consider a point valid
        self.min_area = 10
        
        # Last known aspect ratio of the rectangle
        self.last_aspect_ratio = self.rectangle_width / self.rectangle_height

    def get_led_points(self):
        """Get LED points from the sensor"""
        # Get raw data from sensor
        points_data = self.sensor.get_points()
        
        # Filter points by brightness and area
        valid_points = []
        for point in points_data:
            x, y = point['position']
            # brightness = point['brightness']
            # area = point['area']
            valid_points.append((x, y))

            if len(valid_points) >= 4:
                break
        
        return valid_points

    def calculate_side_lengths(self, points):
        """
        Calculate the side lengths of the quadrilateral formed by the points
        Returns a list of 4 side lengths in clockwise order
        """
        sides = []
        # print(f"side length func got input points {points}")
        # Assuming points are in order: top-left, top-right, bottom-right, bottom-left
        for i in range(4):
            next_i = (i + 1) % 4
            side_length = distance.euclidean(points[i], points[next_i])
            sides.append(side_length)
        return sides

    def identify_rectangle_pattern(self, points):
        """
        Identify the rectangular pattern regardless of orientation
        Returns the points in order: top-left, top-right, bottom-right, bottom-left (clockwise)
        """
        if len(points) != 4:
            print("hi")
            return None
        
        # Create all possible point orderings (24 permutations)
        # But we can optimize by only checking 4 rotations of each pattern
        centroid = np.mean(points, axis=0)
        
        # Sort points by angle around centroid
        angles = []
        for point in points:
            angle = np.arctan2(point[1] - centroid[1], point[0] - centroid[0])
            angles.append(angle)
        
        # Sort points by angle
        sorted_points = [p for _, p in sorted(zip(angles, points), key=lambda pair: pair[0])]
        
        # Generate 4 possible clockwise orderings (rotations)
        possible_orderings = []
        for i in range(4):
            ordering = sorted_points[i:] + sorted_points[:i]
            possible_orderings.append(ordering)
        
        # Find the ordering that best matches a rectangle
        best_ordering = None
        best_score = float('inf')
        
        for ordering in possible_orderings:
            # Calculate side lengths
            sides = self.calculate_side_lengths(ordering)
            
            # In a rectangle, opposite sides should be equal
            # Calculate the difference between opposite sides
            side_diff1 = abs(sides[0] - sides[2])
            side_diff2 = abs(sides[1] - sides[3])
            
            # Calculate aspect ratio
            longer_side = max(sides[0], sides[1])
            shorter_side = min(sides[0], sides[1])
            aspect_ratio = longer_side / shorter_side if shorter_side > 0 else float('inf')
            
            # How close is this to our expected aspect ratio?
            aspect_diff = abs(aspect_ratio - self.last_aspect_ratio)
            
            # Score based on side differences and aspect ratio
            score = side_diff1 + side_diff2 + aspect_diff
            
            if score < best_score:
                best_score = score
                best_ordering = ordering
        
        # If we found a good match, update the last known aspect ratio
        if best_ordering and best_score < 1.0:  # Threshold for "good enough" match
            sides = self.calculate_side_lengths(best_ordering)
            longer_side = max(sides[0], sides[1])
            shorter_side = min(sides[0], sides[1])
            self.last_aspect_ratio = longer_side / shorter_side

        print(f"ur mom {best_ordering}")
        
        return np.array(best_ordering, dtype=np.float32) if best_ordering else None

    def track_points_with_history(self, points):
        """
        Use previous frame information to maintain consistent point identification
        """
        if len(points) != 4 or self.prev_identified_points is None:
            # If we don't have 4 points or no previous data, use geometric identification
            return self.identify_rectangle_pattern(points)
        
        # We have previous points, so try to match current points to them
        # based on proximity
        matched_points = np.zeros((4, 2), dtype=np.float32)
        
        # For each previous point, find the closest current point
        used_indices = set()
        for i, prev_point in enumerate(self.prev_identified_points):
            min_dist = float('inf')
            best_match = -1
            
            for j, curr_point in enumerate(points):
                if j in used_indices:
                    continue
                    
                dist = distance.euclidean(prev_point, curr_point)
                if dist < min_dist:
                    min_dist = dist
                    best_match = j
            
            if best_match != -1:
                matched_points[i] = points[best_match]
                used_indices.add(best_match)
            else:
                # If we couldn't match all points, fall back to geometric method
                return self.identify_rectangle_pattern(points)
        
        # Verify that the matched points still form a reasonable rectangle
        sides = self.calculate_side_lengths(matched_points)
        longer_side = max(sides[0], sides[1])
        shorter_side = min(sides[0], sides[1])
        aspect_ratio = longer_side / shorter_side if shorter_side > 0 else float('inf')
        
        # If aspect ratio is reasonable, accept the tracking result
        if abs(aspect_ratio - self.last_aspect_ratio) < 0.3:  # Threshold for acceptable change
            return matched_points
        else:
            # Otherwise fall back to geometric method
            return self.identify_rectangle_pattern(points)

    def calculate_centroid(self, points):
        """Calculate the centroid of the points"""
        return np.mean(points, axis=0)

    def move_cursor(self, current_centroid):
        """Move cursor based on the relative movement of LED pattern centroid"""
        if self.prev_centroid is not None:
            # Calculate movement delta
            dx = (current_centroid[0] - self.prev_centroid[0]) * self.sensitivity
            dy = (current_centroid[1] - self.prev_centroid[1]) * self.sensitivity
            
            # Invert y-axis if needed for natural cursor movement
            dy = -dy
            
            # Move mouse cursor
            pyautogui.moveRel(int(dx), int(dy))
        
        # Update previous centroid
        self.prev_centroid = current_centroid

    def calibrate(self):
        """Calibrate the system with the current LED positions"""
        points = self.get_led_points()
        identified_points = self.identify_rectangle_pattern(points)
        
        if identified_points is not None:
            self.prev_centroid = self.calculate_centroid(identified_points)
            self.prev_identified_points = identified_points
            self.is_calibrated = True
            
            # Calculate and store the current aspect ratio
            sides = self.calculate_side_lengths(identified_points)
            longer_side = max(sides[0], sides[1])
            shorter_side = min(sides[0], sides[1])
            self.last_aspect_ratio = longer_side / shorter_side
            
            return True
        return False

    def run(self):
        """Main loop to run the cursor controller"""
        print("Starting IR Cursor Controller")
        print("Position the IR LEDs in view and press Enter to calibrate")
        input("Press Enter to calibrate...")
        
        if self.calibrate():
            print("Calibration successful!")
            print(f"Detected rectangle aspect ratio: {self.last_aspect_ratio:.2f}")
        else:
            print("Calibration failed. Ensure all 4 LEDs are visible.")
            return
        
        print("Press Ctrl+C to exit")
        
        try:
            while True:
                # Get LED points
                points = self.get_led_points()
                
                # Process for cursor movement if all LEDs are visible
                if len(points) == 4:
                    identified_points = self.track_points_with_history(points)
                    if identified_points is not None:
                        current_centroid = self.calculate_centroid(identified_points)
                        self.move_cursor(current_centroid)
                        self.prev_identified_points = identified_points
                
                # Small delay to prevent excessive polling
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Exiting...")

# Example sensor interface class
class IRSensor:
    def __init__(self):
        #init serial port
        self.ser = serial.Serial(
        port='COM3', #Ryan: COM3, Luke:/dev/cu.usbserial-0001'
        baudrate=9600,
        timeout=1
        )

        
    def get_points(self):
        # This method should return an array of dictionaries with point data
        # Example return format:
        # [
        #     {'position': (x1, y1), 'brightness': b1, 'area': a1},
        #     {'position': (x2, y2), 'brightness': b2, 'area': a2},
        #     ...
        # ]
        '''
        returns 4 coordinate tuples in a list
        '''
        data = b''
        while data != b'\n':
            data = self.ser.read()
        data = b''
        while data[-1:] != b'\n':
            data += self.ser.read()
        self.ser.reset_input_buffer()
        data = data.decode('utf-8').strip()
        res = data.split(',')
        try:
            res = [int(coord.strip()) for coord in res]
            objs = [
                {'position': (res[0], res[1])},
                {'position': (res[2], res[3])},
                {'position': (res[4], res[5])},
                {'position': (res[6], res[7])}
            ]
            return objs
        except:
            pass

# Example usage
if __name__ == "__main__":
    sensor = IRSensor()  # Replace with your actual sensor initialization
    controller = IRCursorController(sensor)
    controller.run()