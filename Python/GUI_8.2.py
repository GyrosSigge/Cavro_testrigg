import pyqtgraph as pg
import sys, queue, threading, time, serial, csv
from PyQt6 import QtWidgets, QtGui, QtCore
from datetime import datetime
from Fluigent.SDK import fgt_init, fgt_close, fgt_get_sensorChannelsInfo, fgt_get_sensorValue

# ------------------ CONFIG ------------------
STROKE_MM = 40
MAX_POINTS = 1000  # max points per plot
NUM_MOTORS = 10

SERIAL_PORT_CAVRO = "COM3"
SERIAL_BAUD_CAVRO = 38400  # Note that the baudrate has to be defined on cavro usin /U41 for 9600 and U47 for 38400

LOG_FILENAME = f"motor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

ADDRESS = ["1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ";", "<", "=", ">", "?"]

# ------------------ QUEUE & THREADS ------------------
data_queue = queue.Queue()
cavro_cmd_queue = queue.Queue()  # central Cavro command queue
stop_event = threading.Event()

# ------------------ LOG FILE ------------------
log_file = open(LOG_FILENAME, mode='w', newline='')
log_writer = csv.writer(log_file)
header = ['Timestamp']
for i in range(1, NUM_MOTORS+1):
    header += [f'Motor{i}_Pressure', f'Motor{i}_Position']
log_writer.writerow(header)
log_file.flush()

# ------------------ SENSOR INIT (Fluigent) ------------------
fgt_init()
sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()
detected_sensors = {str(sensorInfo.indexID): i for i, sensorInfo in enumerate(sensorInfoArray)}
pressureOffsets = [0.0]*len(sensorInfoArray)  # To store zeroed pressure offsets

# Map sensor indexIDs to motor indices (user-provided mapping)
sensor_map = {'1387328640': 0, '1387336832': 1, '1387320448': 2, '1387185280': 3, '1387345024': 4}

# ------------------ SERIAL (Cavro only) ------------------
ser_cavro = serial.Serial()
ser_cavro.port = SERIAL_PORT_CAVRO
ser_cavro.baudrate = SERIAL_BAUD_CAVRO
ser_cavro.timeout = 1

try:
    ser_cavro.open()
except Exception as e:
    print(f"Error opening Cavro serial port: {e}")

# ------------------ MOTOR DATA ------------------
motor_data = [{"pressure":0.0,"position":0.0} for _ in range(NUM_MOTORS)]

# ------------------ TIME AXIS ------------------
class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [datetime.fromtimestamp(value).strftime("%H:%M:%S") for value in values]

# ------------------ CAVRO SERIAL WORKER ------------------
class SerialCavroWorker(QtCore.QThread):
    plunger_signal = QtCore.pyqtSignal(int, float)  # motor_id, raw plunger value
    valve_signal   = QtCore.pyqtSignal(int, int)    # motor_id, valve pos

    def __init__(self, num_motors:int, parent=None):
        super().__init__(parent)
        self.num_motors = num_motors
        self._running = True

    def run(self):
        while self._running:
            try:
                if ser_cavro.is_open:
                    # 1. Process queued GUI commands first (serialized)
                    try:
                        while True:
                            cmd = cavro_cmd_queue.get_nowait()
                            # cmd already contains trailing \r
                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode(errors="ignore").strip()
                            print(f"Sent {cmd.strip()} | Response: {resp}")
                    except queue.Empty:
                        pass

                    # 2. Poll each motor (plunger + valve)
                    for motor_id in range(self.num_motors):
                        # Plunger
                        try:
                            cmd = f"/{ADDRESS[motor_id]}?\r"
                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode(errors="ignore").strip()
                            if resp.startswith("/0"):
                                raw_pos = float(resp[3:-1])
                                self.plunger_signal.emit(motor_id, raw_pos)
                        except Exception as e:
                            print(f"Cavro plunger read error (motor {motor_id}): {e}")

                        # Valve
                        try:
                            cmd = f"/{ADDRESS[motor_id]}?6\r"
                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode(errors="ignore").strip()
                            if resp.startswith("/0") and not resp.startswith("/0@?"):
                                val = int(resp[3:-1])
                                self.valve_signal.emit(motor_id, val)
                        except Exception as e:
                            print(f"Cavro valve read error (motor {ADDRESS[motor_id]}): {e}")
            except Exception as e:
                print(f"SerialCavroWorker error: {e}")

            #self.msleep(100)

    def stop(self):
        self._running = False
        self.wait()

# ------------------ MOTOR PANEL ------------------
class MotorPanel(QtWidgets.QWidget):
    def __init__(self, motor_id):
        super().__init__()
        self.motor_id = motor_id
        self.valve_position = 3
        self.plunger_position = 0.0

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        # Motor label
        self.label = QtWidgets.QLabel(f"Motor {motor_id+1}")
        main_layout.addWidget(self.label)

        # Buttons
        btn_layout = QtWidgets.QGridLayout()
        main_layout.addLayout(btn_layout)

        self.btn_init = QtWidgets.QPushButton("Init")
        self.btn_up   = QtWidgets.QPushButton("Move Up")
        self.btn_down = QtWidgets.QPushButton("Move Down")
        self.btn_valve_in  = QtWidgets.QPushButton("Valve Input")
        self.btn_valve_out = QtWidgets.QPushButton("Valve Output")
        self.btn_valve_ext = QtWidgets.QPushButton("Valve Extra")
        self.btn_loop = QtWidgets.QPushButton("Start Loop")
        self.btn_stop = QtWidgets.QPushButton("Stop")

        btn_layout.addWidget(self.btn_init, 0, 0)
        btn_layout.addWidget(self.btn_up,   1, 0)
        btn_layout.addWidget(self.btn_down, 2, 0)
        btn_layout.addWidget(self.btn_loop, 3, 0)
        btn_layout.addWidget(self.btn_valve_in, 0, 1)
        btn_layout.addWidget(self.btn_valve_out,1, 1)
        btn_layout.addWidget(self.btn_valve_ext,2, 1)
        btn_layout.addWidget(self.btn_stop, 3, 1)

        # Connect buttons
        self.btn_init.clicked.connect(self.btn_init_func)
        self.btn_up.clicked.connect(lambda: send_message(self.motor_id, "A0R"))
        self.btn_down.clicked.connect(lambda: send_message(self.motor_id, "A3000R"))
        self.btn_valve_in.clicked.connect(lambda: send_message(self.motor_id, "IR"))
        self.btn_valve_out.clicked.connect(lambda: send_message(self.motor_id, "OR"))
        self.btn_valve_ext.clicked.connect(lambda: send_message(self.motor_id, "ER"))
        self.btn_loop.clicked.connect(self.btn_loop_func)
        self.btn_stop.clicked.connect(lambda: send_message(self.motor_id, "T"))

        # Status layout
        status_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(status_layout)

        labels_layout = QtWidgets.QVBoxLayout()
        self.status_pressure = QtWidgets.QLabel("Pressure: 0.0")
        self.status_position = QtWidgets.QLabel("Position: 0.0")
        labels_layout.addWidget(self.status_pressure)
        labels_layout.addWidget(self.status_position)
        labels_layout.addStretch()
        status_layout.addLayout(labels_layout)

        img_bar_layout = QtWidgets.QVBoxLayout()
        self.image_label = QtWidgets.QLabel()
        self.set_image("valve_output.png")
        img_bar_layout.addWidget(self.image_label, alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

        self.position_bar = QtWidgets.QProgressBar()
        self.position_bar.setOrientation(QtCore.Qt.Orientation.Vertical)
        self.position_bar.setRange(0, STROKE_MM)
        self.position_bar.setValue(STROKE_MM)
        self.position_bar.setTextVisible(False)
        self.position_bar.setFixedHeight(30)
        img_bar_layout.addWidget(self.position_bar, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        status_layout.addLayout(img_bar_layout)

        # Plots
        self.plot_pressure = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Pressure (mbar)")
        self.plot_position = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Position (mm)")

        main_layout.addWidget(self.plot_pressure)
        main_layout.addWidget(self.plot_position)

        self.curve_pressure = self.plot_pressure.plot(pen='r')
        self.curve_position = self.plot_position.plot(pen='b')

        # Data storage
        self.timestamps = []
        self.pressures = []
        self.positions = []

    def set_image(self, filename):
        pixmap = QtGui.QPixmap(filename).scaled(50, 50, QtCore.Qt.AspectRatioMode.KeepAspectRatio, QtCore.Qt.TransformationMode.SmoothTransformation)
        pixmap.setDevicePixelRatio(self.devicePixelRatio())
        self.image_label.setPixmap(pixmap)

    def btn_init_func(self):
        send_message(self.motor_id, "ZR")

    def btn_loop_func(self):
        try:
            w.max_pressure = int(w.entry_max_pressure.text())
            w.start_pos = int(w.entry_start.text())
            w.stop_pos = int(w.entry_stop.text())
            w.loop_cycles = int(w.entry_cycles.text())
        except ValueError:
            print("Invalid pressure/pos/cycles value. Please enter integers.")
            return
        message = f"g,M500,O,M500,A{w.start_pos},M500,I,M500,A{w.stop_pos},G{w.loop_cycles},M500,O,M500,A{w.start_pos}R,"
        send_message(self.motor_id, message)

    def add_data_point(self, p, pos, timestamp):
        self.timestamps.append(timestamp.timestamp())
        self.pressures.append(p)
        self.positions.append(pos)
        self.timestamps = self.timestamps[-MAX_POINTS:]
        self.pressures = self.pressures[-MAX_POINTS:]
        self.positions = self.positions[-MAX_POINTS:]
        motor_data[self.motor_id]['pressure'] = p
        motor_data[self.motor_id]['position'] = pos

    def update_plot(self):
        if self.timestamps:
            self.curve_pressure.setData(self.timestamps, self.pressures)
            self.curve_position.setData(self.timestamps, self.positions)
            self.plot_pressure.setYRange(min(min(self.pressures), -1), max(max(self.pressures), 1))
            self.plot_position.setYRange(min(min(self.positions), -1), 0)

        self.status_pressure.setText(f"Pressure: {motor_data[self.motor_id]['pressure']:.1f}")
        self.status_position.setText(f"Position: {motor_data[self.motor_id]['position']:.1f}")
        self.position_bar.setValue(int(STROKE_MM + (motor_data[self.motor_id]['position'])))

    def update_plunger_position(self, raw_pos: float):
        # Convert raw Cavro plunger value to mm. Adjust conversion if needed for your hardware.
        pos_mm = - raw_pos / 3000.0 * STROKE_MM
        motor_data[self.motor_id]['position'] = pos_mm

    def update_valve_position(self, valve_p: int):
        self.valve_position = valve_p
        if valve_p == 1:
            self.set_image("valve_input.png")
        elif valve_p == 2:
            self.set_image("valve_extra.png")
        elif valve_p == 3:
            self.set_image("valve_output.png")

# ------------------ MAIN WINDOW ------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Panel")

        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)

        # Main vertical layout
        main_vlayout = QtWidgets.QVBoxLayout(central_widget)
        main_vlayout.setContentsMargins(8, 8, 8, 8)
        main_vlayout.setSpacing(8)
        

        # ================= CONTROL AREA =================
        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setSpacing(12)
        main_vlayout.addLayout(controls_layout)

        # ===== Actions =====
        action_box = QtWidgets.QGroupBox("Actions")
        action_layout = QtWidgets.QVBoxLayout(action_box)
        self.btn_loop_all   = QtWidgets.QPushButton("Start Loop All")
        self.btn_zero_all_p = QtWidgets.QPushButton("Zero All Pressures")
        self.btn_stop_all = QtWidgets.QPushButton("STOP ALL")
        self.btn_stop_all.setStyleSheet("background-color: red; color: white; font-weight: bold")
        action_layout.addWidget(self.btn_loop_all)
        action_layout.addWidget(self.btn_zero_all_p)
        action_layout.addWidget(self.btn_stop_all)
        controls_layout.addWidget(action_box)

        # ===== Loop Parameters =====
        loop_box = QtWidgets.QGroupBox("Loop Parameters")
        loop_layout = QtWidgets.QFormLayout(loop_box)
        self.entry_cycles = QtWidgets.QLineEdit("10")
        self.entry_start = QtWidgets.QLineEdit("0")
        self.entry_stop  = QtWidgets.QLineEdit("3000")
        loop_layout.addRow("Cycles:", self.entry_cycles)
        loop_layout.addRow("Top Pos:", self.entry_start)
        loop_layout.addRow("Bottom Pos:", self.entry_stop)
        controls_layout.addWidget(loop_box)

        # ===== Settings =====
        self.freq = 0
        settings_box = QtWidgets.QGroupBox("Settings")
        settings_layout = QtWidgets.QGridLayout(settings_box)

        self.entry_max_pressure = QtWidgets.QLineEdit("7000")
        self.btn_set_pressure_max = QtWidgets.QPushButton("Apply")
        settings_layout.addWidget(QtWidgets.QLabel("Max/Stop\nPressure [mbar]:"), 0, 0)
        settings_layout.addWidget(self.entry_max_pressure, 0, 1)
        settings_layout.addWidget(self.btn_set_pressure_max, 0, 2)

        self.entry_set_freq = QtWidgets.QLineEdit("50")
        self.btn_set_freq = QtWidgets.QPushButton("Apply")
        settings_layout.addWidget(QtWidgets.QLabel("Data Sample Freq (Hz):"), 1, 0)
        settings_layout.addWidget(self.entry_set_freq, 1, 1)
        settings_layout.addWidget(self.btn_set_freq, 1, 2)

        self.frequency_text = QtWidgets.QLabel("Actual Freq:")
        self.frequency_text.setStyleSheet("color: rgb(200, 200, 200);")
        self.frequency = QtWidgets.QLabel("  0.0")
        self.frequency.setStyleSheet("color: rgb(200, 200, 200);")

        settings_layout.addWidget(self.frequency_text, 2, 0)
        settings_layout.addWidget(self.frequency, 2, 1)

        controls_layout.addWidget(settings_box)

        # ================= CONNECTIONS =================
        self.btn_zero_all_p.clicked.connect(self.btn_zero_all_p_func)
        self.btn_stop_all.clicked.connect(self.btn_stop_all_func)
        self.btn_loop_all.clicked.connect(self.btn_loop_all_func)
        self.btn_set_pressure_max.clicked.connect(self.set_pressure_max_func)
        self.btn_set_freq.clicked.connect(self.btn_set_freq_func)

        # Initialize parameters
        try:
            self.max_pressure = int(self.entry_max_pressure.text())
        except Exception:
            self.max_pressure = 7000
        try:
            self.start_pos = int(self.entry_start.text())
        except Exception:
            self.start_pos = 0
        try:
            self.stop_pos = int(self.entry_stop.text())
        except Exception:
            self.stop_pos = 3000
        try:
            self.loop_cycles = int(self.entry_cycles.text())
        except Exception:
            self.loop_cycles = 10

        # ================= BOTTOM AREA =================
        top_line = self._line(vertical=False)
        top_line.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed)
        main_vlayout.addWidget(top_line)

        # Use a grid for motor panels
        panels_grid = QtWidgets.QGridLayout()
        panels_grid.setSpacing(8)
        main_vlayout.addLayout(panels_grid)

        self.panels = [MotorPanel(i) for i in range(NUM_MOTORS)]

        # place up to 5 panels per row
        max_per_row = 5
        for i, panel in enumerate(self.panels):
            row = i // max_per_row
            col = i % max_per_row
            panels_grid.addWidget(panel, row, col)
            panel.update_plot()

        # ================= TIMERS =================
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_all_panels)
        self.update_timer.start(50)  # UI refresh 20 Hz

        # sampling frequency and timer
        self.sampling_freq = float(self.entry_set_freq.text())
        self.data_timer = QtCore.QTimer(self)
        self.data_timer.timeout.connect(self.sample_and_queue)
        self.data_timer.start(int(1000.0 / max(self.sampling_freq, 1)))

        # Cavro worker
        self.cavro_worker = SerialCavroWorker(NUM_MOTORS)
        self.cavro_worker.plunger_signal.connect(self.on_plunger_update)
        self.cavro_worker.valve_signal.connect(self.on_valve_update)
        self.cavro_worker.start()

    # -------- Cavro worker slots --------
    def on_plunger_update(self, motor_id:int, raw_pos:float):
        self.panels[motor_id].update_plunger_position(raw_pos)

    def on_valve_update(self, motor_id:int, valve_p:int):
        self.panels[motor_id].update_valve_position(valve_p)

    # -------- UI callbacks --------
    def set_pressure_max_func(self):
        try:
            self.max_pressure = int(self.entry_max_pressure.text())
        except ValueError:
            print("Invalid max pressure value. Please enter an integer.")
            return
        print(f"Max pressure set to {self.max_pressure} mbar")

    def btn_zero_all_p_func(self):
        global pressureOffsets
        # Recompute offsets from current motor_data pressures
        # Map sensor readings to motor indices using sensor_map
        for sensorInfo in sensorInfoArray:
            try:
                sensor_id = str(sensorInfo.indexID)
                if sensor_id in sensor_map:
                    idx = sensor_map[sensor_id]
                    if idx < NUM_MOTORS:
                        pressureOffsets[idx] = pressureOffsets[idx] + motor_data[idx]['pressure']
            except Exception as e:
                print(f"Error zeroing sensor {sensorInfo.index}: {e}")
        print("Zeroed pressures:", pressureOffsets)

    def btn_stop_all_func(self):
        message = ""
        for i in range(NUM_MOTORS):
            message += f"/{ADDRESS[i]}T,"
        send_message(None, message, RAW=True)

    def btn_loop_all_func(self):
        try:
            self.max_pressure = int(self.entry_max_pressure.text())
            self.start_pos = int(self.entry_start.text())
            self.stop_pos = int(self.entry_stop.text())
            self.loop_cycles = int(self.entry_cycles.text())
        except ValueError:
            print("Invalid pressure/pos/cycles value. Please enter integers.")
            return
        message = ""
        for i in range(NUM_MOTORS):
            message += f"/{ADDRESS[i]}"
            message += f"M{500*i},"
            message += f"g,M500,O,M500,A{self.start_pos},M500,I,M500,A{self.stop_pos},G{self.loop_cycles},M500,O,M500,A{self.start_pos}R,"
        send_message(None, message, RAW=True)

    def btn_set_freq_func(self):
        try:
            freq_local = float(self.entry_set_freq.text())
            if freq_local <= 0:
                raise ValueError()
            self.sampling_freq = freq_local
            self.data_timer.start(int(1000.0 / self.sampling_freq))
            print(f"Sampling freq set to {self.sampling_freq} Hz")
        except Exception:
            print("Invalid frequency value. Please enter a positive number.")

    # ---------------- DATA SAMPLING ----------------
    def sample_and_queue(self):
        # Read Fluigent sensors for pressures
        pressures = [0.0]*NUM_MOTORS
        for sensorInfo in sensorInfoArray:
            try:
                measurement = fgt_get_sensorValue(sensorInfo.index)
                sensor_id = str(sensorInfo.indexID)
                if sensor_id in sensor_map:
                    idx = sensor_map[sensor_id]
                    if idx < NUM_MOTORS:
                        pressures[idx] = measurement - pressureOffsets[idx]
            except Exception as e:
                print(f"Error reading Fluigent sensor {sensorInfo.index}: {e}")

        # Positions come from motor_data (updated by Cavro worker)
        positions = [motor_data[i]['position'] for i in range(NUM_MOTORS)]

        timestamp = datetime.now()
        data_queue.put((pressures, positions, timestamp))

        # Log CSV
        row = [timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')]
        for i in range(NUM_MOTORS):
            row += [pressures[i], positions[i]]
        log_writer.writerow(row)

        # update freq estimate (exponential smoothing)
        now = time.time()
        if not hasattr(self, '_last_sample_time'):
            self._last_sample_time = now
            self._smoothed_dt = 1.0/self.sampling_freq if self.sampling_freq>0 else 1.0
        else:
            dt = now - self._last_sample_time
            self._last_sample_time = now
            alpha = 0.05
            self._smoothed_dt = alpha * dt + (1-alpha) * self._smoothed_dt
        self.freq = 1.0 / self._smoothed_dt if self._smoothed_dt > 0 else 0.0

    # ---------------- UI UPDATES ----------------
    def update_all_panels(self):
        try:
            while True:
                pressures, positions, timestamp = data_queue.get_nowait()
                for i, panel in enumerate(self.panels):
                    panel.add_data_point(pressures[i], positions[i], timestamp)

                    if pressures[i] >= self.max_pressure:
                        send_message(i, "T")
        except queue.Empty:
            pass

        for panel in self.panels:
            panel.update_plot()

        # Display freq
        self.frequency.setText(f"  {self.freq:.1f} Hz")

    def _line(self, vertical=False):
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.Shape.VLine if vertical else QtWidgets.QFrame.Shape.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        return line

    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            if hasattr(self, 'cavro_worker'):
                self.cavro_worker.stop()
        finally:
            event.accept()

# ------------------ SEND MESSAGE ------------------
def send_message(m_id: int, cmd: str, RAW: bool=False) -> None:
    # Build full command with trailing CR and enqueue for cavro_worker to execute
    if RAW:
        # If RAW True, user provided a string that may already contain slashes; ensure \r
        full_cmd = cmd if cmd.endswith('\r') else cmd + '\r'
    else:
        # m_id may be None when sending RAW batched commands; guard against that
        full_cmd = f"/{ADDRESS[m_id]}{cmd}\r"
    cavro_cmd_queue.put(full_cmd)
    print(f"Enqueued: {full_cmd.strip()}")

# ------------------ MAIN ------------------
if __name__ == "__main__":
    print("===================== START =====================")
    print("Detected Sensors:", detected_sensors)
    print("Mapped Sensors:", sensor_map)
    print("Starting GUI...")

    # Initialize Cavro motors (safe; enqueued)
    if ser_cavro.is_open:
        try:
            for i in range(NUM_MOTORS):
                msg = f"/{ADDRESS[i]}"
                msg += f"M{2500*i}"
                msg += "ZR\r"
                cavro_cmd_queue.put(msg)
        except Exception:
            pass

    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()

    try:
        sys.exit(app.exec())
    finally:
        stop_event.set()
        try:
            if hasattr(w, 'cavro_worker'):
                w.cavro_worker.stop()
        except Exception:
            pass
        try:
            ser_cavro.close()
        except Exception:
            pass
        try:
            fgt_close()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass
        print("Program exited cleanly.")