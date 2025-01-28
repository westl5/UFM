import sys
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

serial_port = 'COM4'
baud_rate = 9600  

class RealTimePlot(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Set up main window
        self.setWindowTitle("Real-Time Serial Data Plot")
        self.setGeometry(100, 100, 800, 600)

        # Create a plot widget
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Y Coordinate')
        self.plot_widget.setLabel('bottom', 'X Coordinate')
        self.plot_widget.setXRange(0, 4095)
        self.plot_widget.setYRange(0, 4095)

        # Initialize data storage
        self.x_data = []
        self.y_data = []

        # Create plot curve
        self.curve = self.plot_widget.plot(pen='r')

        # Set up serial connection
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)

        # Timer to read and update plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)  # Update every 16 ms

    def parse_coordinates(self, data):
        data = data.split(',')
        data = [float(i) for i in data]
        coordinates = [(data[0],data[1]),(data[2],data[3]),(data[4],data[5]),(data[6],data[7])]

        return coordinates

    def update_plot(self):
        if self.ser.in_waiting > 0:
            data = b''
            while data != b'\n':
                data = self.ser.read()
            data = b''
            while data[-1:] != b'\n':
                data += self.ser.read()
            self.ser.reset_input_buffer()
            data = data.decode('utf-8').strip()

            # Parse coordinates
            coordinates = self.parse_coordinates(data)
            if not coordinates:
                return

            # Update data storage
            self.x_data = [x for x, y in coordinates]
            self.y_data = [y for x, y in coordinates]

            # Update plot curve
            self.curve.setData(self.x_data, self.y_data)

    def closeEvent(self, event):
        self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = RealTimePlot()
    window.show()
    sys.exit(app.exec_())