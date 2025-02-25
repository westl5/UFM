import pyautogui
import time
from dataclasses import dataclass
from screeninfo import get_monitors

@dataclass
class MousePosition:
    x: float
    y: float

class MouseController:
    def __init__(self):
        # Get primary monitor resolution
        monitor = get_monitors()[0]
        self.screen_width = monitor.width
        self.screen_height = monitor.height
        
        # Disable PyAutoGUI's fail-safe
        pyautogui.FAILSAFE = False
        
        # Movement parameters
        self.smooth_factor = 0.3 # Adjust for more/less smoothing
        self.last_x = self.screen_width / 2
        self.last_y = self.screen_height / 2
    
    def move_to(self, x: float, y: float, smooth: bool = True):
        """
        Move mouse to specified coordinates
        
        Args:
            x: X coordinate (0 to screen width)
            y: Y coordinate (0 to screen height)
            smooth: Whether to apply smoothing
        """
        # Ensure coordinates are within screen bounds
        x = max(0, min(x, self.screen_width))
        y = max(0, min(y, self.screen_height))
        
        if smooth:
            # Apply smoothing
            smoothed_x = (x * self.smooth_factor + 
                         self.last_x * (1 - self.smooth_factor))
            smoothed_y = (y * self.smooth_factor + 
                         self.last_y * (1 - self.smooth_factor))
            
            # Update last position
            self.last_x = smoothed_x
            self.last_y = smoothed_y
            
            pyautogui.moveTo(smoothed_x, smoothed_y)
        else:
            self.last_x = x
            self.last_y = y
            pyautogui.moveTo(x, y, _pause=False)
    
    def move_relative(self, dx: float, dy: float):
        """Move mouse relative to current position"""
        current_x, current_y = pyautogui.position()
        self.move_to(current_x + dx, current_y + dy)
    
    def get_position(self) -> MousePosition:
        """Get current mouse position"""
        x, y = pyautogui.position()
        return MousePosition(x, y)
    
    def left_click(self, duration: float = 0.1):
        """Perform left click"""
        pyautogui.click(duration=duration)
    
    def right_click(self, duration: float = 0.1):
        """Perform right click"""
        pyautogui.rightClick(duration=duration)
    
    def double_click(self, duration: float = 0.1):
        """Perform double click"""
        pyautogui.doubleClick(duration=duration)
    
    def scroll(self, clicks: int):
        """
        Scroll the mouse wheel
        
        Args:
            clicks: Number of clicks (positive = up, negative = down)
        """
        pyautogui.scroll(clicks)

# Example usage
def main():
    # Create controller
    mouse = MouseController()
    
    # Example movements
    print("Moving mouse in a square pattern...")
    
    # Move to corners
    corners = [
        (100, 100),
        (500, 100),
        (500, 500),
        (100, 500)
    ]
    
    for x, y in corners:
        mouse.move_to(x, y)
        time.sleep(1)  # Pause to see movement
        mouse.left_click()
        time.sleep(0.5)
    
    # Example relative movement
    print("Moving in relative patterns...")
    for _ in range(4):
        mouse.move_relative(50, 0)  # Right
        time.sleep(0.5)
        mouse.move_relative(0, 50)  # Down
        time.sleep(0.5)
    
    # Get current position
    pos = mouse.get_position()
    print(f"Final position: ({pos.x}, {pos.y})")

if __name__ == "__main__":
    main()