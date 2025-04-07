import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QTextEdit,
    QGroupBox, QMessageBox, QSlider
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
            
            # Query current sensitivity on connect
            self.serial.write(b"get_sensitivity\n")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to connect: {str(e)}")

    def disconnect(self):
        if self.serial:
            self.serial.close()
        self.connect_btn.setText("Connect")
        QMessageBox.information(self, "Disconnected", "Serial port closed")

class SensitivityControl(QGroupBox):
    def __init__(self, serial_widget):
        super().__init__("Sensitivity Configuration")
        self.serial_widget = serial_widget
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        
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
        
        # Button layout
        button_layout = QHBoxLayout()
        self.configure_btn = QPushButton("Apply")
        self.configure_btn.clicked.connect(self.send_configuration)
        self.reset_btn = QPushButton("Reset to Default")
        self.reset_btn.clicked.connect(self.reset_sensitivity)
        
        button_layout.addWidget(self.configure_btn)
        button_layout.addWidget(self.reset_btn)
        
        # Add all layouts to main layout
        layout.addLayout(input_layout)
        layout.addLayout(slider_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def update_from_slider(self):
        # Convert slider value (1-100) to sensitivity (0.1-10.0)
        value = round(self.sens_slider.value() / 10, 1)
        self.sens_input.setText(str(value))

    def reset_sensitivity(self):
        self.sens_input.setText("2.0")
        self.sens_slider.setValue(20)
        self.send_configuration()

    def send_configuration(self):
        if not self.serial_widget.serial or not self.serial_widget.serial.is_open:
            QMessageBox.warning(self, "Error", "Not connected to serial port!")
            return

        try:
            value = float(self.sens_input.text())
            if 0.1 <= value <= 10.0:
                command = f"sensitivity={value}\n"
                self.serial_widget.serial.write(command.encode())
            else:
                QMessageBox.warning(self, "Invalid Value", 
                                  "Value must be between 0.1 and 10.0")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", 
                              "Please enter a valid number")

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
                        # Update UI if sensitivity value is reported
                        if data.startswith("Sensitivity set to:"):
                            try:
                                value = float(data.split(":")[-1].strip())
                                # Find the sensitivity control and update it
                                parent = self.parent()
                                if parent and hasattr(parent, "sensitivity_control"):
                                    parent.sensitivity_control.sens_input.setText(str(value))
                                    parent.sensitivity_control.sens_slider.setValue(int(value * 10))
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
        self.setGeometry(100, 100, 700, 500)
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