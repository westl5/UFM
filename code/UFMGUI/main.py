import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QTextEdit,
    QGroupBox, QMessageBox, QSlider, QSpinBox, QGridLayout
)
from PyQt6.QtCore import QTimer, Qt
from serial import Serial
from serial.tools.list_ports import comports

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
        self.baud_label = QLabel("Baud Rate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200"])
        self.baud_combo.setCurrentText("115200")  # Match ESP32 baud rate
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        layout.addWidget(self.port_label)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.baud_label)
        layout.addWidget(self.baud_combo)
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
            baud_rate = int(self.baud_combo.currentText())
            self.serial = Serial(port, baud_rate, timeout=1)
            self.connect_btn.setText("Disconnect")
            QMessageBox.information(self, "Connected", f"Connected to {port} at {baud_rate} baud")
            
            # Query current settings on connect
            self.serial.write(b"get_sensitivity\n")
            self.serial.write(b"get_left_click_angle\n")
            self.serial.write(b"get_right_click_angle\n")
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
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        
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
        self.right_min_spin.setRange(-120, -10)
        self.right_min_spin.setValue(-80)  # Default value
        
        self.right_max_label = QLabel("Max Angle:")
        self.right_max_spin = QSpinBox()
        self.right_max_spin.setRange(-120, -10)
        self.right_max_spin.setValue(-50)  # Default value
        
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
        layout.addWidget(sens_group)
        layout.addWidget(left_click_group)
        layout.addWidget(right_click_group)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def update_from_slider(self):
        # Convert slider value (1-100) to sensitivity (0.1-10.0)
        value = round(self.sens_slider.value() / 10, 1)
        self.sens_input.setText(str(value))

    def reset_settings(self):
        # Reset sensitivity
        self.sens_input.setText("2.0")
        self.sens_slider.setValue(20)
        
        # Reset left click angles
        self.left_min_spin.setValue(50)
        self.left_max_spin.setValue(80)
        
        # Reset right click angles
        self.right_min_spin.setValue(-80)
        self.right_max_spin.setValue(-50)
        
        # Apply the changes
        self.send_configuration()

    def send_configuration(self):
        if not self.serial_widget.serial or not self.serial_widget.serial.is_open:
            QMessageBox.warning(self, "Error", "Not connected to serial port!")
            return

        try:
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
            right_min = self.right_min_spin.value()
            right_max = self.right_max_spin.value()
            if right_min <= right_max:
                QMessageBox.warning(self, "Invalid Range", 
                                  "Right click minimum angle must be greater than maximum angle (since they're negative)")
                return
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
                            # Handle sensitivity updates
                            if data.startswith("Sensitivity set to:"):
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
                                    parent.sensitivity_control.right_min_spin.setValue(value)
                                except:
                                    pass
                            elif data.startswith("Right click max angle:"):
                                try:
                                    value = int(data.split(":")[-1].strip())
                                    parent.sensitivity_control.right_max_spin.setValue(value)
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
        self.setWindowTitle("BLE Mouse Sensitivity Configurator")
        self.setGeometry(100, 100, 700, 600)
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        layout = QVBoxLayout()

        # Serial Configuration
        self.serial_widget = SerialConfigWidget()
        
        # Sensitivity Control
        self.sensitivity_control = SensitivityControl(self.serial_widget)
        
        # Console
        self.console = ConsoleWidget(self.serial_widget)
        self.console.setParent(self)  # Set parent for communication

        layout.addWidget(self.serial_widget)
        layout.addWidget(self.sensitivity_control)
        layout.addWidget(self.console)
        
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def closeEvent(self, event):
        if self.serial_widget.serial and self.serial_widget.serial.is_open:
            self.serial_widget.serial.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())