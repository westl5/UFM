import sys
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QTextEdit,
    QGroupBox, QMessageBox, QSlider, QSpinBox, QGridLayout
)
from PyQt6.QtCore import QTimer, Qt, QSize, QPropertyAnimation, QEasingCurve, QPoint, QRect
from PyQt6.QtGui import QFont, QPixmap, QPainter, QMovie
from serial import Serial
from serial.tools.list_ports import comports

class AnimatedLogoWidget(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScaledContents(True)
        
        pixmap = QPixmap('gengy.png')
        self.setPixmap(pixmap.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio))

class ProductImageWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Create image label for product image
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        mouse_image_path = "Generate.png"  # Replace with your actual image path
        if os.path.exists(mouse_image_path):
            pixmap = QPixmap(mouse_image_path)
            self.image_label.setPixmap(pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio))
        
        layout.addWidget(self.image_label)
        
        self.setLayout(layout)

class SerialConfigWidget(QGroupBox):
    def __init__(self):
        super().__init__("Serial Configuration")
        self.serial = None
        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        layout = QHBoxLayout()
        
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        layout.addWidget(self.port_label)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_btn)
        layout.addWidget(self.connect_btn)
        
        self.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in comports()]
        if not ports:
            self.port_combo.addItem("No ports found")
        else:
            self.port_combo.addItems(ports)

    def toggle_connection(self):
        if self.serial and self.serial.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_combo.currentText()
        if not port or port == "No ports found":
            QMessageBox.warning(self, "Error", "No valid port selected")
            return

        try:
            baud_rate = 9600  # Permanently set to 9600
            self.serial = Serial(port, baud_rate, timeout=1)
            self.connect_btn.setText("Disconnect")
            QMessageBox.information(self, "Connected", f"Connected to {port} at {baud_rate} baud")
            
            # Query current settings on connect
            self.serial.write(b"get_sensitivity\n")
            self.serial.write(b"get_left_click_angle\n")
            self.serial.write(b"get_right_click_angle\n")
            self.serial.write(b"get_handedness\n")
            self.serial.write(b"get_click_enabled\n")  # Add query for click enabled status
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to connect: {str(e)}")

    def disconnect(self):
        if self.serial:
            self.serial.close()
        self.connect_btn.setText("Connect")
        QMessageBox.information(self, "Disconnected", "Serial port closed")

class SensitivityControl(QGroupBox):
    def __init__(self, serial_widget):
        super().__init__("Configuration")
        self.serial_widget = serial_widget
        self.is_right_handed = True  # Default to right-handed mode
        self.is_clicking_enabled = True  # Default to clicking enabled
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        
        # Handedness mode selection
        handedness_group = QGroupBox("Mouse Mode")
        handedness_layout = QHBoxLayout()
        
        # Create a mode label
        self.mode_label = QLabel("Select Mode:")
        
        # Create two separate buttons for handedness selection
        self.right_handed_btn = QPushButton("RIGHT-HANDED")
        self.right_handed_btn.setMinimumWidth(120)
        self.right_handed_btn.setCheckable(True)
        self.right_handed_btn.clicked.connect(self.set_right_handed)
        
        self.left_handed_btn = QPushButton("LEFT-HANDED")
        self.left_handed_btn.setMinimumWidth(120)
        self.left_handed_btn.setCheckable(True)
        self.left_handed_btn.clicked.connect(self.set_left_handed)
        
        # Set a bold font for both buttons
        font = QFont()
        font.setBold(True)
        self.right_handed_btn.setFont(font)
        self.left_handed_btn.setFont(font)
        
        # Set initial button states
        self.update_handedness_buttons()
        
        handedness_layout.addWidget(self.mode_label)
        handedness_layout.addWidget(self.right_handed_btn)
        handedness_layout.addWidget(self.left_handed_btn)
        handedness_layout.addStretch()
        
        handedness_group.setLayout(handedness_layout)
        
        # Click Mode Toggle
        click_mode_group = QGroupBox("Click Mode")
        click_mode_layout = QHBoxLayout()
        
        self.click_mode_label = QLabel("Click Mode:")
        self.click_toggle_btn = QPushButton("ENABLED")
        self.click_toggle_btn.setMinimumWidth(120)
        self.click_toggle_btn.setCheckable(True)
        self.click_toggle_btn.setChecked(True)
        self.click_toggle_btn.clicked.connect(self.toggle_click_mode)
        
        # Apply the same bold font
        self.click_toggle_btn.setFont(font)
        
        # Add active style to the button initially
        self.update_click_button()
        
        click_mode_layout.addWidget(self.click_mode_label)
        click_mode_layout.addWidget(self.click_toggle_btn)
        click_mode_layout.addStretch()
        
        click_mode_group.setLayout(click_mode_layout)
        
        # Sensitivity Configuration
        sens_group = QGroupBox("Sensitivity")
        sens_layout = QVBoxLayout()
        
        # Text input for precise entry
        input_layout = QHBoxLayout()
        self.sens_label = QLabel("Sensitivity (0.1-10.0):")
        self.sens_input = QLineEdit("2.0")  # Default value
        self.sens_input.setMaximumWidth(100)
        
        input_layout.addWidget(self.sens_label)
        input_layout.addWidget(self.sens_input)
        input_layout.addStretch()
        
        # Slider for easier adjustment
        slider_layout = QHBoxLayout()
        self.slider_label = QLabel("Adjust:")
        self.sens_slider = QSlider(Qt.Orientation.Horizontal)
        self.sens_slider.setMinimum(1)
        self.sens_slider.setMaximum(100)
        self.sens_slider.setValue(20)  # Default 2.0
        self.sens_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.sens_slider.setTickInterval(10)
        
        # Connect slider movement to text update
        self.sens_slider.valueChanged.connect(self.update_from_slider)
        
        slider_layout.addWidget(self.slider_label)
        slider_layout.addWidget(self.sens_slider)
        
        sens_layout.addLayout(input_layout)
        sens_layout.addLayout(slider_layout)
        sens_group.setLayout(sens_layout)
        
        # Left Click Angle Configuration
        left_click_group = QGroupBox("Left Click Angle")
        left_click_layout = QGridLayout()
        
        self.left_min_label = QLabel("Min Angle:")
        self.left_min_spin = QSpinBox()
        self.left_min_spin.setRange(10, 120)
        self.left_min_spin.setValue(50)  # Default value
        
        self.left_max_label = QLabel("Max Angle:")
        self.left_max_spin = QSpinBox()
        self.left_max_spin.setRange(10, 120)
        self.left_max_spin.setValue(80)  # Default value
        
        left_click_layout.addWidget(self.left_min_label, 0, 0)
        left_click_layout.addWidget(self.left_min_spin, 0, 1)
        left_click_layout.addWidget(self.left_max_label, 0, 2)
        left_click_layout.addWidget(self.left_max_spin, 0, 3)
        
        left_click_group.setLayout(left_click_layout)
        
        # Right Click Angle Configuration
        right_click_group = QGroupBox("Right Click Angle")
        right_click_layout = QGridLayout()

        self.right_min_label = QLabel("Min Angle:")
        self.right_min_spin = QSpinBox()
        self.right_min_spin.setRange(10, 120)  # Positive range
        self.right_min_spin.setValue(80)  # Default value (abs of -80)

        self.right_max_label = QLabel("Max Angle:")
        self.right_max_spin = QSpinBox()
        self.right_max_spin.setRange(10, 120)  # Positive range
        self.right_max_spin.setValue(50)  # Default value (abs of -50)

        right_click_layout.addWidget(self.right_min_label, 0, 0)
        right_click_layout.addWidget(self.right_min_spin, 0, 1)
        right_click_layout.addWidget(self.right_max_label, 0, 2)
        right_click_layout.addWidget(self.right_max_spin, 0, 3)

        right_click_group.setLayout(right_click_layout)
        
        # Button layout
        button_layout = QHBoxLayout()
        self.configure_btn = QPushButton("Apply All Settings")
        self.configure_btn.clicked.connect(self.send_configuration)
        self.reset_btn = QPushButton("Reset to Defaults")
        self.reset_btn.clicked.connect(self.reset_settings)
        
        button_layout.addWidget(self.configure_btn)
        button_layout.addWidget(self.reset_btn)
        
        # Add all layouts to main layout
        layout.addWidget(handedness_group)
        layout.addWidget(click_mode_group)  # Add the new click mode group
        layout.addWidget(sens_group)
        layout.addWidget(left_click_group)
        layout.addWidget(right_click_group)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def update_from_slider(self):
        # Convert slider value (1-100) to sensitivity (0.1-10.0)
        value = round(self.sens_slider.value() / 10, 1)
        self.sens_input.setText(str(value))

    def set_right_handed(self):
        if not self.is_right_handed:
            self.is_right_handed = True
            self.update_handedness_buttons()
            self.send_handedness_setting()

    def set_left_handed(self):
        if self.is_right_handed:
            self.is_right_handed = False
            self.update_handedness_buttons()
            self.send_handedness_setting()
    
    def toggle_click_mode(self):
        self.is_clicking_enabled = not self.is_clicking_enabled
        self.update_click_button()
        self.send_click_mode_setting()
    
    def update_click_button(self):
        # Active button style - dark background with white text
        active_style = "background-color: #505050; color: white; font-weight: bold;"
        # Inactive button style - greyed out
        inactive_style = "background-color: #e0e0e0; color: #909090;"
        
        if self.is_clicking_enabled:
            self.click_toggle_btn.setText("ENABLED")
            self.click_toggle_btn.setStyleSheet(active_style)
        else:
            self.click_toggle_btn.setText("DISABLED")
            self.click_toggle_btn.setStyleSheet(inactive_style)
    
    def send_click_mode_setting(self):
        # If connected, send the click mode setting immediately
        if self.serial_widget.serial and self.serial_widget.serial.is_open:
            enabled = "true" if self.is_clicking_enabled else "false"
            command = f"click_enabled={enabled}\n"
            self.serial_widget.serial.write(command.encode())
    
    def update_handedness_buttons(self):
        # Active button style - dark background with white text
        active_style = "background-color: #505050; color: white; font-weight: bold;"
        # Inactive button style - greyed out
        inactive_style = "background-color: #e0e0e0; color: #909090;"
        
        if self.is_right_handed:
            self.right_handed_btn.setChecked(True)
            self.right_handed_btn.setStyleSheet(active_style)
            self.left_handed_btn.setChecked(False)
            self.left_handed_btn.setStyleSheet(inactive_style)
        else:
            self.left_handed_btn.setChecked(True)
            self.left_handed_btn.setStyleSheet(active_style)
            self.right_handed_btn.setChecked(False)
            self.right_handed_btn.setStyleSheet(inactive_style)

    def send_handedness_setting(self):
        # If connected, send the new handedness setting immediately
        if self.serial_widget.serial and self.serial_widget.serial.is_open:
            mode = "right" if self.is_right_handed else "left"
            command = f"handedness={mode}\n"
            self.serial_widget.serial.write(command.encode())

    def reset_settings(self):
        # Reset handedness to right-handed
        self.is_right_handed = True
        self.update_handedness_buttons()
        
        # Reset click mode to enabled
        self.is_clicking_enabled = True
        self.update_click_button()
        
        # Reset sensitivity
        self.sens_input.setText("2.0")
        self.sens_slider.setValue(20)
        
        # Reset left click angles
        self.left_min_spin.setValue(50)
        self.left_max_spin.setValue(80)
        
        # Reset right click angles (using positive values)
        self.right_min_spin.setValue(80)  # abs(-80)
        self.right_max_spin.setValue(50)  # abs(-50)
        
        # Apply the changes
        self.send_configuration()

    def send_configuration(self):
        if not self.serial_widget.serial or not self.serial_widget.serial.is_open:
            QMessageBox.warning(self, "Error", "Not connected to serial port!")
            return

        try:
            # Send handedness setting
            mode = "right" if self.is_right_handed else "left"
            command = f"handedness={mode}\n"
            self.serial_widget.serial.write(command.encode())
            
            # Send click mode setting
            enabled = "true" if self.is_clicking_enabled else "false"
            command = f"click_enabled={enabled}\n"
            self.serial_widget.serial.write(command.encode())
            
            # Send sensitivity
            value = float(self.sens_input.text())
            if 0.1 <= value <= 10.0:
                command = f"sensitivity={value}\n"
                self.serial_widget.serial.write(command.encode())
            else:
                QMessageBox.warning(self, "Invalid Value", 
                                "Sensitivity must be between 0.1 and 10.0")
                return  
                
            # Validate and send left click angle range
            left_min = self.left_min_spin.value()
            left_max = self.left_max_spin.value()
            if left_min >= left_max:
                QMessageBox.warning(self, "Invalid Range", 
                                "Left click minimum angle must be less than maximum angle")
                return
            command = f"left_click_min={left_min}\n"
            self.serial_widget.serial.write(command.encode())
            command = f"left_click_max={left_max}\n"
            self.serial_widget.serial.write(command.encode())
            
            # Validate and send right click angle range
            # Get positive values from UI
            right_min_positive = self.right_min_spin.value()
            right_max_positive = self.right_max_spin.value()
            
            # Convert to negative values for internal use
            right_min = -right_min_positive
            right_max = -right_max_positive
            
            # Validate (after conversion)
            if right_min <= right_max:
                QMessageBox.warning(self, "Invalid Range", 
                                "Right click minimum angle must be greater than maximum angle")
                return
                
            # Send negative values to the device
            command = f"right_click_min={right_min}\n"
            self.serial_widget.serial.write(command.encode())
            command = f"right_click_max={right_max}\n"
            self.serial_widget.serial.write(command.encode())
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", 
                            "Please enter a valid number for sensitivity")
class ConsoleWidget(QGroupBox):
    def __init__(self, serial_widget):
        super().__init__("Console")
        self.serial_widget = serial_widget
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(100)

    def init_ui(self):
        layout = QVBoxLayout()
        
        # Console output
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        
        # Button to clear console
        button_layout = QHBoxLayout()
        self.clear_btn = QPushButton("Clear Console")
        self.clear_btn.clicked.connect(self.clear_console)
        button_layout.addWidget(self.clear_btn)
        button_layout.addStretch()
        
        layout.addWidget(self.console)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def clear_console(self):
        self.console.clear()

    def read_serial(self):
        if self.serial_widget.serial and self.serial_widget.serial.is_open:
            try:
                while self.serial_widget.serial.in_waiting:
                    data = self.serial_widget.serial.readline().decode().strip()
                    if data:
                        self.console.append(f"<< {data}")
                        # Update UI if settings values are reported
                        parent = self.parent()
                        if parent and hasattr(parent, "sensitivity_control"):
                            # Handle handedness updates
                            if data.startswith("Handedness set to:"):
                                mode = data.split(":")[-1].strip().lower()
                                parent.sensitivity_control.is_right_handed = (mode == "right")
                                parent.sensitivity_control.update_handedness_buttons()
                            
                            # Handle click mode updates
                            elif data.startswith("Click mode set to:"):
                                enabled = data.split(":")[-1].strip().lower()
                                parent.sensitivity_control.is_clicking_enabled = (enabled == "enabled")
                                parent.sensitivity_control.update_click_button()
                            
                            # Handle sensitivity updates
                            elif data.startswith("Sensitivity set to:"):
                                try:
                                    value = float(data.split(":")[-1].strip())
                                    parent.sensitivity_control.sens_input.setText(str(value))
                                    parent.sensitivity_control.sens_slider.setValue(int(value * 10))
                                except:
                                    pass
                            # Handle left click angle updates
                            elif data.startswith("Left click min angle:"):
                                try:
                                    value = int(data.split(":")[-1].strip())
                                    parent.sensitivity_control.left_min_spin.setValue(value)
                                except:
                                    pass
                            elif data.startswith("Left click max angle:"):
                                try:
                                    value = int(data.split(":")[-1].strip())
                                    parent.sensitivity_control.left_max_spin.setValue(value)
                                except:
                                    pass
                            # Handle right click angle updates
                            elif data.startswith("Right click min angle:"):
                                try:
                                    value = int(data.split(":")[-1].strip())
                                    # Convert negative value to positive for display
                                    parent.sensitivity_control.right_min_spin.setValue(abs(value))
                                except:
                                    pass
                            elif data.startswith("Right click max angle:"):
                                try:
                                    value = int(data.split(":")[-1].strip())
                                    # Convert negative value to positive for display
                                    parent.sensitivity_control.right_max_spin.setValue(abs(value))
                                except:
                                    pass
            except Exception as e:
                self.console.append(f"<< ERROR: {str(e)}")
                if self.serial_widget.serial and self.serial_widget.serial.is_open:
                    self.serial_widget.serial.close()
                    self.serial_widget.connect_btn.setText("Connect")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UFM Configurator")
        self.setGeometry(100, 100, 800, 750)  # Made window larger to accommodate new elements
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # Header with logo
        header_layout = QHBoxLayout()
        
        # Add animated logo
        self.logo = AnimatedLogoWidget()
        header_layout.addWidget(self.logo)
        
        # Add title next to logo
        title_label = QLabel("UFM Configurator")
        title_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #2c3e50;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        
        main_layout.addLayout(header_layout)
        
        # Main content layout (three columns now: product image, controls, and console)
        content_layout = QHBoxLayout()
        
        # Left panel - Product image
        self.product_image = ProductImageWidget()
        
        # Middle panel - Controls
        controls_layout = QVBoxLayout()
        
        # Serial Configuration
        self.serial_widget = SerialConfigWidget()
        
        # Sensitivity Control
        self.sensitivity_control = SensitivityControl(self.serial_widget)
        
        controls_layout.addWidget(self.serial_widget)
        controls_layout.addWidget(self.sensitivity_control)
        
        # Create a widget to hold the controls layout
        controls_widget = QWidget()
        controls_widget.setLayout(controls_layout)
        
        # Right panel - Console
        self.console = ConsoleWidget(self.serial_widget)
        self.console.setParent(self)  # Set parent for communication
        
        # Add all three panels to content layout
        # content_layout.addWidget(self.product_image)
        content_layout.addWidget(controls_widget)
        content_layout.addWidget(self.console)
        
        # Set a stretch factor to control relative widths
        content_layout.setStretchFactor(self.product_image, 1)
        content_layout.setStretchFactor(controls_widget, 2)
        content_layout.setStretchFactor(self.console, 1)
        
        main_layout.addLayout(content_layout)
        
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def closeEvent(self, event):
        if self.serial_widget.serial and self.serial_widget.serial.is_open:
            self.serial_widget.serial.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Apply some basic styling to the entire app
    app.setStyle("Fusion")
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec())