from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import queue
import threading
import time
from datetime import datetime
import serial
import csv
from Fluigent.SDK import fgt_init, fgt_close, fgt_get_sensorChannelsInfo, fgt_get_sensorValue

# ------------------ CONFIG ------------------
STROKE_MM = 40
MAX_POINTS = 1000  # max points per plot
NUM_MOTORS = 5

SERIAL_PORT_CAVRO = "COM3"
SERIAL_BAUD_CAVRO = 9600
SERIAL_PORT_ESP32 = "COM4"
SERIAL_BAUD_ESP32 = 115200

LOG_FILENAME = f"motor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ------------------ QUEUE & THREADS ------------------
data_queue = queue.Queue()
stop_event = threading.Event()

# ------------------ LOG FILE ------------------
log_file = open(LOG_FILENAME, mode='w', newline='')
log_writer = csv.writer(log_file)
header = ['Timestamp']
for i in range(1, NUM_MOTORS+1):
    header += [f'Motor{i}_Pressure', f'Motor{i}_Current', f'Motor{i}_Position']
log_writer.writerow(header)
log_file.flush()

# ------------------ SENSOR INIT ------------------
fgt_init()
sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()
detected_sensors = {str(sensorInfo.indexID): i for i, sensorInfo in enumerate(sensorInfoArray)}

sensor_map = {
    "1387328640": 0,  # Motor slot 1
    "1387185280": 1,  # Motor slot 2
    # "1387328640": 2,  # Motor slot 2
    # "1387328640": 3,  # Motor slot 2
    # "1387328640": 4,  # Motor slot 2
    # Add more SensorIDs here as needed
}

# ------------------ SERIAL ------------------
ser_cavro = serial.Serial(SERIAL_PORT_CAVRO, SERIAL_BAUD_CAVRO, timeout=1)
ser_esp32 = serial.Serial(SERIAL_PORT_ESP32, SERIAL_BAUD_ESP32, timeout=1)

# ------------------ MOTOR DATA ------------------
motor_data = [{"pressure":0.0,"current":0.0,"position":0.0} for _ in range(NUM_MOTORS)]

# ------------------ TIME AXIS ------------------
class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        """Convert float timestamps to HH:MM:SS"""
        return [datetime.fromtimestamp(value).strftime("%H:%M:%S") for value in values]
    
# ------------------ MOTOR PANEL ------------------
class MotorPanel(QtWidgets.QWidget):
    def __init__(self, motor_id):
        super().__init__()
        self.motor_id = motor_id

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # Labels
        self.label = QtWidgets.QLabel(f"Motor {motor_id+1}")
        layout.addWidget(self.label)

        # Buttons
        btn_layout = QtWidgets.QGridLayout()
        layout.addLayout(btn_layout)

        self.btn_init = QtWidgets.QPushButton("Init")
        self.btn_up   = QtWidgets.QPushButton("Move Up")
        self.btn_down = QtWidgets.QPushButton("Move Down")
        self.btn_valve_in  = QtWidgets.QPushButton("Valve Input")
        self.btn_valve_out = QtWidgets.QPushButton("Valve Output")
        self.btn_valve_ext = QtWidgets.QPushButton("Valve Extra")

        btn_layout.addWidget(self.btn_init, 0, 0)
        btn_layout.addWidget(self.btn_up,   1, 0)
        btn_layout.addWidget(self.btn_down, 2, 0)
        btn_layout.addWidget(self.btn_valve_in, 0, 1)
        btn_layout.addWidget(self.btn_valve_out,1, 1)
        btn_layout.addWidget(self.btn_valve_ext,2, 1)

        # Connect buttons
        self.btn_init.clicked.connect(lambda: send_message(self.motor_id, "ZR"))
        self.btn_up.clicked.connect(lambda: send_message(self.motor_id, "A3000R"))
        self.btn_down.clicked.connect(lambda: send_message(self.motor_id, "A1000R"))
        self.btn_valve_in.clicked.connect(lambda: send_message(self.motor_id, "IR"))
        self.btn_valve_out.clicked.connect(lambda: send_message(self.motor_id, "OR"))
        self.btn_valve_ext.clicked.connect(lambda: send_message(self.motor_id, "ER"))

        # Status labels
        self.status_pressure = QtWidgets.QLabel("Pressure: 0.0")
        self.status_current  = QtWidgets.QLabel("Current: 0.0")
        self.status_position = QtWidgets.QLabel("Position: 0.0")
        layout.addWidget(self.status_pressure)
        layout.addWidget(self.status_current)
        layout.addWidget(self.status_position)

        # Plots (with custom time axis)
        self.plot_pressure = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Pressure (mbar)")
        self.plot_current  = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Current (mA)")
        self.plot_position = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Position (mm)")


        layout.addWidget(self.plot_pressure)
        layout.addWidget(self.plot_current)
        layout.addWidget(self.plot_position)

        # Curves
        self.curve_pressure = self.plot_pressure.plot(pen='r')
        self.curve_current  = self.plot_current.plot(pen='g')
        self.curve_position = self.plot_position.plot(pen='b')

        # Data storage
        self.timestamps = []
        self.pressures = []
        self.currents  = []
        self.positions = []

    def add_data_point(self, p, c, pos, timestamp):
        """Store new values for plotting"""
        self.timestamps.append(timestamp.timestamp())
        self.pressures.append(p)
        self.currents.append(c)
        self.positions.append(pos)

        # Trim history
        self.timestamps = self.timestamps[-MAX_POINTS:]
        self.pressures = self.pressures[-MAX_POINTS:]
        self.currents = self.currents[-MAX_POINTS:]
        self.positions = self.positions[-MAX_POINTS:]

        # Update motor_data
        motor_data[self.motor_id]['pressure'] = p
        motor_data[self.motor_id]['current'] = c
        motor_data[self.motor_id]['position'] = pos

    def update_plot(self):
        """Refresh curves + labels"""
        if self.timestamps:
            self.curve_pressure.setData(self.timestamps, self.pressures)
            self.curve_current.setData(self.timestamps, self.currents)
            self.curve_position.setData(self.timestamps, self.positions)

            # --- Always include zero ---
            self.plot_pressure.setYRange(0, max(max(self.pressures), 1))  # avoid zero-height
            self.plot_current.setYRange(0, max(max(self.currents), 1))
            self.plot_position.setYRange(0, max(max(self.positions), 1))

        self.status_pressure.setText(f"Pressure: {motor_data[self.motor_id]['pressure']:.1f}")
        self.status_current.setText(f"Current: {motor_data[self.motor_id]['current']:.1f}")
        self.status_position.setText(f"Position: {motor_data[self.motor_id]['position']:.1f}")

# ------------------ MAIN WINDOW ------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("5-Motor Control Panel")
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(layout)

        self.panels = [MotorPanel(i) for i in range(NUM_MOTORS)]
        for panel in self.panels:
            layout.addWidget(panel)

        # 🔴 ONE central timer for all panels
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_all_panels)
        self.update_timer.start(50)  # 20 Hz

    def update_all_panels(self):
        """Pull all available queue data and update panels in sync"""
        try:
            while True:
                pressures, currents, positions, timestamp = data_queue.get_nowait()
                for i, panel in enumerate(self.panels):
                    panel.add_data_point(pressures[i], currents[i], positions[i], timestamp)
        except queue.Empty:
            pass

        # Update all panels' plots & labels
        for panel in self.panels:
            panel.update_plot()

# ------------------ SERIAL / SENSOR THREAD ------------------
def read_sensors():
    last_print_time = time.time()

    while not stop_event.is_set():
        # ESP32
        line = ser_esp32.readline().decode().strip()
        if not line:
            continue
        values = line.split(",")
        if len(values) < NUM_MOTORS*2:
            continue
        currents = list(map(float, values[:NUM_MOTORS]))
        currents = [-i for i in currents]
        positions = list(map(float, values[NUM_MOTORS:NUM_MOTORS*2]))

        # Fluigent pressures
        pressures = [0.0]*NUM_MOTORS
        for sensorInfo in sensorInfoArray:
            measurement = fgt_get_sensorValue(sensorInfo.index)
            sensor_id = str(sensorInfo.indexID)
            if sensor_id in sensor_map:
                idx = sensor_map[sensor_id]
                pressures[idx] = measurement

        # Enqueue once for all panels
        data_queue.put((pressures, currents, positions, datetime.now()))

        # Log CSV
        row = [datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')]
        for i in range(NUM_MOTORS):
            row += [pressures[i], currents[i], positions[i]]
        log_writer.writerow(row)

        # Flush periodically
        now = time.time()
        if now - last_print_time >= 5:
            log_file.flush()
            last_print_time = now


# ------------------ SEND MESSAGE ------------------
def send_message(m_id, cmd: str):
    full_cmd = f"/{m_id+2}{cmd}\r"
    ser_cavro.write(full_cmd.encode())
    print(full_cmd)

# ------------------ MAIN ------------------
if __name__ == "__main__":
    print("################################")
    print("Detected Sensors:", detected_sensors)
    print("Mapped Sensors:", sensor_map)
    print("Starting GUI...")
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()

    # Start sensor reading thread
    t = threading.Thread(target=read_sensors, daemon=True)
    t.start()

    try:
        sys.exit(app.exec())
    finally:
        stop_event.set()
        t.join()
        ser_cavro.close()
        ser_esp32.close()
        fgt_close()
        log_file.close()
        print("Program exited cleanly.")
