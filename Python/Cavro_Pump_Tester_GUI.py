import pyqtgraph as pg
import sys, queue, threading, time, serial, csv
from PyQt6 import QtWidgets, QtGui, QtCore
from datetime import datetime
from Fluigent.SDK import fgt_init, fgt_close, fgt_get_sensorChannelsInfo, fgt_get_sensorValue
from collections import deque
import serial.tools.list_ports

# ==================== CONFIG ====================
# ================================================

NUM_PUMPS = 6

SERIAL_PORT_CAVRO_FALLBACK = "COM3" # Fallback if find_cavro_port() fails
SERIAL_BAUD_CAVRO = 38400  # Cavro baud

LOG_FILENAME = f"Pump_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Map sensor indexIDs to Pump indices (user-provided mapping)
SENSOR_MAP = {'1387328640': 0, '1387336832': 1, '1387320448': 2, '1387185280': 3, '1387345024': 4, '1387332736': 5}

STROKE_MM = 40 # Convert Cavro steps to mm
STROKE_INCREMENTS = 3000 # Steps or increments in a stroke
PLOT_MAX_POINTS = 500  # max points per plot

# Cavro pump adresses
ADDRESS = ["1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ";", "<", "=", ">", "?"]


# =============== CONFIG END ===============
# ==========================================


# ------------------ QUEUE & THREADS ------------------
# Keep a very small queue (only latest sample) so put() won't block
data_queue = queue.Queue(maxsize=1)
cavro_cmd_queue = queue.Queue()
stop_event = threading.Event()

# ------------------ LOG FILE ------------------
log_file = open(LOG_FILENAME, mode='w', newline='')
log_writer = csv.writer(log_file)
header = ['Timestamp']
for i in range(1, NUM_PUMPS+1):
    header += [f'Pump{i}_#Cycles, Pump{i}_Pressure', f'Pump{i}_Pressure_Flag', f'Pump{i}_Plunger_Position', f'Pump{i}_Valve_Position']
log_writer.writerow(header)
log_file.flush()
log_lock = threading.Lock()

# ------------------ SENSOR INIT (Fluigent) ------------------
fgt_init()
sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()
detected_sensors = {str(sensorInfo.indexID): i for i, sensorInfo in enumerate(sensorInfoArray)}
pressureOffsets = [0.0]*len(sensorInfoArray)  # To store zeroed pressure offsets

# ------------------ SERIAL (Cavro only) ------------------
def find_cavro_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # You can inspect port attributes to filter
        print(f"Checking {port.device}: {port.description}")
        
        ser = serial.Serial(port.device, SERIAL_BAUD_CAVRO, timeout=1)
        # Try sending a Cavro-specific command or handshake here
        # Example: Cavro devices often respond to "/1?" with "OK"
        ser.write(b"/1?\r")
        response = ser.readline().decode(errors="ignore").strip()
        ser.close()

        if response.startswith("/0"):
            print(f"Cavro pump found on {port.device}")
            return port.device
        else:
            print(f"No Cavro pump found... Using fallback {SERIAL_PORT_CAVRO_FALLBACK}")
            return SERIAL_PORT_CAVRO_FALLBACK
        
ser_cavro = serial.Serial()
ser_cavro.port = find_cavro_port()
ser_cavro.baudrate = SERIAL_BAUD_CAVRO
ser_cavro.timeout = 1

try:
    ser_cavro.open()
except Exception as e:
    print(f"Error opening Cavro serial port: {e}")



# ------------------ Pump DATA ------------------
Pump_data_lock = threading.Lock()
Pump_data = [{"pressure":0.0,"position":0.0,"valve":0, "cycles":0} for _ in range(NUM_PUMPS)]

# ------------------ TIME AXIS ------------------
class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [datetime.fromtimestamp(value).strftime('%H:%M:%S') for value in values]

# ------------------ CAVRO SERIAL WORKER ------------------
class SerialCavroWorker(QtCore.QThread):
    plunger_signal = QtCore.pyqtSignal(int, float)  # Pump_id, raw plunger value
    valve_signal   = QtCore.pyqtSignal(int, int)    # Pump_id, valve pos

    def __init__(self, NUM_PUMPS:int, parent=None):
        super().__init__(parent)
        self.NUM_PUMPS = NUM_PUMPS
        self._running = True
        self.counter = 0

    def run(self):
        while self._running:
            try:
                if ser_cavro.is_open:
                    # process queued GUI commands first
                    try:
                        while True:
                            cmd = cavro_cmd_queue.get_nowait()
                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode(errors='ignore').strip()
                            # optional: could emit ack signal
                    except queue.Empty:
                        pass

                    # Read plunger for each Pump (fast)
                    for Pump_id in range(self.NUM_PUMPS):
                        try:
                            cmd = f"/{ADDRESS[Pump_id]}?\r"
                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode(errors='ignore').strip()
                            if resp.startswith('/0'):
                                # robust parsing
                                raw = ''.join(ch for ch in resp[3:] if (ch.isdigit() or ch=='.' or ch=='-'))
                                try:
                                    raw_pos = float(raw)
                                    self.plunger_signal.emit(Pump_id, raw_pos)
                                except Exception:
                                    pass
                        except Exception as e:
                            print(f"Cavro plunger read error (Pump {Pump_id}): {e}")

                    # Read valves less often to reduce serial traffic
                    if self.counter >= 5:
                        for Pump_id in range(self.NUM_PUMPS):
                            try:
                                cmd = f"/{ADDRESS[Pump_id]}?6\r"
                                ser_cavro.write(cmd.encode())
                                resp = ser_cavro.read_until().decode(errors='ignore').strip()
                                if resp.startswith('/0') and not resp.startswith('/0@?'):
                                    raw = ''.join(ch for ch in resp[3:] if ch.isdigit())
                                    if raw:
                                        val = int(raw)
                                        self.valve_signal.emit(Pump_id, val)
                            except Exception as e:
                                print(f"Cavro valve read error (Pump {Pump_id}): {e}")
                        self.counter = 0
                    else:
                        self.counter += 1
            except Exception as e:
                print(f"SerialCavroWorker error: {e}")

    def stop(self):
        self._running = False
        self.wait()


# ------------------ SAMPLING WORKER ------------------
class SamplingWorker(QtCore.QThread):
    freq_signal = QtCore.pyqtSignal(float)  # emit actual commit frequency

    def __init__(self, sample_freq: float = 10.0, max_pressure=7000, parent=None):
        super().__init__(parent)
        self.max_pressure = max_pressure
        self._running = True
        self.flags = [False] * NUM_PUMPS
        self.cycle_counter = [0] * NUM_PUMPS
        self.restarts = [0] * NUM_PUMPS
        self.set_rate(sample_freq)  # ensure clamping and setup

        # --- Error capture setup ---
        self.error_buffer = deque(maxlen=50 * 10)   # keep last 10 sec of data at 50Hz
        self.error_active = [False] * NUM_PUMPS   # whether currently capturing after trigger
        self.error_capture = [[] for _ in range(NUM_PUMPS)]
        self.error_after_samples = int(50 * 10)     # capture 10 sec after trigger
        self.error_remaining = [0] * NUM_PUMPS

    def set_rate(self, hz: float):
        # Clamp frequency between 1 and 50 Hz
        if hz < 1.0:
            hz = 1.0
        if hz > 50.0:
            hz = 50.0
        self.sample_freq = hz
        self.commit_period = 1.0 / self.sample_freq

    def set_max_pressure(self, max_p: float):
        self.max_pressure = max_p

    def run(self):
        loop_period = 1.0 / 50.0  # fixed loop at 50 Hz
        next_commit_time = time.time()

        freq_update_start = time.time()
        commit_count = 0

        low_start_times = [None] * NUM_PUMPS 

        last_valves = [Pump_data[i]['valve'] for i in range(NUM_PUMPS)]

        while self._running:
            start = time.time()
            pressures = [0.0] * NUM_PUMPS

            # read Fluigent sensors
            for sensorInfo in sensorInfoArray:
                try:
                    measurement = fgt_get_sensorValue(sensorInfo.index)
                    sensor_id = str(sensorInfo.indexID)
                    if sensor_id in SENSOR_MAP:
                        idx = SENSOR_MAP[sensor_id]
                        if idx < NUM_PUMPS:
                            pressures[idx] = measurement - pressureOffsets[idx]
                            Pump_data[idx]['pressure'] = pressures[idx]

                            # --- Safety: stop if pressure too high ---
                            if pressures[idx] >= self.max_pressure:
                                send_message(idx, "T")
                                if self.flags[idx] == False:
                                    self.flags[idx] = True
                                    low_start_times[idx] = None
                                    print(f"Warning Pump{idx+1} exceeded max pressure, P={pressures[idx]}\nSaving error data (10s)...")
                            else:
                                if self.flags[idx] == True:
                                    if low_start_times[idx] is None:
                                        low_start_times[idx] = time.time()
                                    elif (time.time() - low_start_times[idx]) >= 1.0:
                                        low_start_times[idx] = None
                                        self.flags[idx] = False  # reset after restart
                except Exception as e:
                    print(f"Error reading Fluigent sensor {sensorInfo.index}: {e}")
           
            with Pump_data_lock:
                positions = [Pump_data[i]['position'] for i in range(NUM_PUMPS)]
                valves = [Pump_data[i]['valve'] for i in range(NUM_PUMPS)]
            timestamp = datetime.now()

            for i in range(NUM_PUMPS):
                if valves[i] == 1 and last_valves[i] == 2:
                    self.cycle_counter[i] += 1
                    Pump_data[i]['cycles'] = self.cycle_counter[i]
            last_valves = valves

            # Row structure for both logging and error capture
            row = [timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')]
            for i in range(NUM_PUMPS):
                row += [self.cycle_counter[i], pressures[i], self.flags[i], positions[i], valves[i]]

            # --- Store in rolling error buffer (always 50 Hz) ---
            self.error_buffer.append(row)

            # --- Check for flag events ---
            for i in range(NUM_PUMPS):
                if not self.error_active[i]:
                    # Trigger: False -> True
                    if self.flags[i]:
                        self.error_active[i] = True
                        # Start capture with current buffer contents
                        self.error_capture[i] = list(self.error_buffer)
                        self.error_remaining[i] = self.error_after_samples
                else:
                    # Continue capturing "after" samples
                    self.error_capture[i].append(row)
                    self.error_remaining[i] -= 1
                    if self.error_remaining[i] <= 0:
                        # Save to CSV
                        fname = f"error_pump{i+1}_{timestamp.strftime('%Y%m%d_%H%M%S')}.csv"
                        with open(fname, "w", newline="") as f:
                            writer = csv.writer(f)
                            writer.writerow(header)
                            writer.writerows(self.error_capture[i])
                        print(f"Error data saved: {fname}")
                        # Reset
                        self.error_active[i] = False
                        self.error_capture[i] = []

                        pump_cycles = int(w.entry_cycles.text())
                        start_pos = w.start_pos
                        stop_pos = w.stop_pos
                        pump_max_restarts = w.max_restarts
                        if self.restarts[i] < pump_max_restarts:
                            message = f"/{ADDRESS[i]}Y,g,M500,E,M500,A{start_pos},M500,I,M500,A{stop_pos},G{pump_cycles - self.cycle_counter[i]},M500,E,M500,A{start_pos}R"
                            send_message(None, message, RAW=True)
                            self.restarts[i] += 1
                            print(f"Restarting Pump {i+1} [{pump_cycles - self.cycle_counter[i]} of {pump_cycles} remaining cycles, {pump_max_restarts-self.restarts[i]} of {pump_max_restarts} remaining restarts]")
                        else:
                            print(f"Max Restarts achieved for Pump {i+1}...")
                            self.restarts[i] = 0

            # --- Commit to main CSV / queue at user rate ---
            now = time.time()
            if now >= next_commit_time:
                # Queue write
                try:
                    data_queue.put_nowait((pressures, positions, valves, timestamp))
                except queue.Full:
                    try:
                        _ = data_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        data_queue.put_nowait((pressures, positions, valves, timestamp))
                    except Exception:
                        pass

                # CSV write
                with log_lock:
                    log_writer.writerow(row)
                    log_file.flush()

                commit_count += 1
                next_commit_time += self.commit_period
                # If we fell behind, skip ahead instead of stalling
                while next_commit_time < now:
                    next_commit_time += self.commit_period

            # --- Update effective commit frequency once per second ---
            if (now - freq_update_start) >= 1.0:
                avg_freq = commit_count / (now - freq_update_start)
                self.freq_signal.emit(avg_freq)
                commit_count = 0
                freq_update_start = now

            # --- Sleep to maintain fixed 50 Hz loop ---
            elapsed = time.time() - start
            to_sleep = loop_period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    def stop(self):
        self._running = False
        self.wait()



# ------------------ Pump PANEL ------------------
class PumpPanel(QtWidgets.QWidget):
    def __init__(self, Pump_id):
        super().__init__()
        self.Pump_id = Pump_id
        self.valve_position = 3
        self.plunger_position = 0.0

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        # Pump label
        self.label = QtWidgets.QLabel(f"Pump {Pump_id+1}")
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
        self.btn_up.clicked.connect(lambda: send_message(self.Pump_id, "A0R"))
        self.btn_down.clicked.connect(lambda: send_message(self.Pump_id, "A3000R"))
        self.btn_valve_in.clicked.connect(lambda: send_message(self.Pump_id, "IR"))
        self.btn_valve_out.clicked.connect(lambda: send_message(self.Pump_id, "OR"))
        self.btn_valve_ext.clicked.connect(lambda: send_message(self.Pump_id, "ER"))
        self.btn_loop.clicked.connect(self.btn_loop_func)
        self.btn_stop.clicked.connect(lambda: send_message(self.Pump_id, "T"))

        # Status layout
        status_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(status_layout)

        labels_layout = QtWidgets.QVBoxLayout()
        self.status_pressure = QtWidgets.QLabel("Pressure (mbar):\t0.0")
        self.status_position = QtWidgets.QLabel("Position (mm):\t0.0")
        self.status_cycles = QtWidgets.QLabel("Cycles (#):\t0")
        labels_layout.addWidget(self.status_pressure)
        labels_layout.addWidget(self.status_position)
        labels_layout.addWidget(self.status_cycles)
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
        send_message(self.Pump_id, "YR")

    def btn_loop_func(self):
        try:
            w.max_pressure = int(w.entry_max_pressure.text())
            w.start_pos = int(w.entry_start.text())
            w.start_pos = int(w.start_pos / STROKE_MM * STROKE_INCREMENTS)
            w.stop_pos = int(w.entry_stop.text())
            w.stop_pos = int(w.stop_pos / STROKE_MM * STROKE_INCREMENTS)
            w.max_restarts = int(w.entry_max_restarts.text())
            w.loop_cycles = int(w.entry_cycles.text())
        except ValueError:
            print("Invalid pressure/pos/cycles value. Please enter integers.")
            return
        message = f"Y,g,M500,E,M500,A{w.start_pos},M500,I,M500,A{w.stop_pos},G{w.loop_cycles},M500,E,M500,A{w.start_pos}R,"
        send_message(self.Pump_id, message)

    def add_data_point(self, p, pos, timestamp):
        self.timestamps.append(timestamp.timestamp())
        self.pressures.append(p)
        self.positions.append(pos)
        self.timestamps = self.timestamps[-PLOT_MAX_POINTS:]
        self.pressures = self.pressures[-PLOT_MAX_POINTS:]
        self.positions = self.positions[-PLOT_MAX_POINTS:]
        # Pump_data updated elsewhere

    def update_plot(self):
        if self.timestamps:
            self.curve_pressure.setData(self.timestamps, self.pressures)
            self.curve_position.setData(self.timestamps, self.positions)
            try:
                self.plot_pressure.setYRange(min(min(self.pressures), -1), max(max(self.pressures), 1))
                self.plot_position.setYRange(min(min(self.positions), -1), 0)
            except Exception:
                pass

        with Pump_data_lock:
            p = Pump_data[self.Pump_id]['pressure']
            pos = Pump_data[self.Pump_id]['position']
            cycl = Pump_data[self.Pump_id]['cycles']
        self.status_pressure.setText(f"Pressure (mbar):\t{p:.1f}")
        self.status_position.setText(f"Position (mm):\t{pos:.1f}")
        self.status_cycles.setText(f"Cycles (#):\t{cycl}")
        self.position_bar.setValue(int(STROKE_MM + (pos)))

    def update_plunger_position(self, raw_pos: float):
        # Convert raw Cavro plunger value to mm. Adjust conversion if needed for your hardware.
        pos_mm = - raw_pos / 3000.0 * STROKE_MM
        with Pump_data_lock:
            Pump_data[self.Pump_id]['position'] = pos_mm

    def update_valve_position(self, valve_p: int):
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
        self.setWindowTitle("Pump Control Panel")

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
        self.entry_stop  = QtWidgets.QLineEdit(f"{STROKE_MM}")
        self.entry_max_restarts  = QtWidgets.QLineEdit(f"{3}")
        loop_layout.addRow("Cycles in Loop:", self.entry_cycles)
        loop_layout.addRow("Top Position (mm):", self.entry_start)
        loop_layout.addRow("Bottom Position (mm):", self.entry_stop)
        loop_layout.addRow("Max Restarts:\n(Restart after max pressure stop)", self.entry_max_restarts)
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

        self.entry_set_freq = QtWidgets.QLineEdit("10")
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
            self.stop_pos = STROKE_MM
        try:
            self.loop_cycles = int(self.entry_cycles.text())
        except Exception:
            self.loop_cycles = 10
        try:
            self.max_restarts = int(self.entry_max_restarts.text())
        except Exception:
            self.max_restarts = 10

        # ================= BOTTOM AREA =================
        top_line = self._line(vertical=False)
        top_line.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed)
        main_vlayout.addWidget(top_line)

        # Use a grid for Pump panels
        panels_grid = QtWidgets.QGridLayout()
        panels_grid.setSpacing(8)
        main_vlayout.addLayout(panels_grid)

        self.panels = [PumpPanel(i) for i in range(NUM_PUMPS)]

        # place up to 5 panels per row
        max_per_row = 5
        for i, panel in enumerate(self.panels):
            row = i // max_per_row
            col = i % max_per_row
            panels_grid.addWidget(panel, row, col)
            panel.update_plot()

        # ================= TIMERS =================
        # GUI update timer (plot/redraw) - keep low rate so GUI remains responsive
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_all_panels)
        self.update_timer.start(100)  # UI refresh 10 Hz

        # Start workers
        self.cavro_worker = SerialCavroWorker(NUM_PUMPS)
        self.cavro_worker.plunger_signal.connect(self.on_plunger_update)
        self.cavro_worker.valve_signal.connect(self.on_valve_update)
        self.cavro_worker.start()

        # Sampling worker (separate thread) - handles Fluigent reads, sampling, and CSV
        self.sampling_freq = float(self.entry_set_freq.text())
        self.sampler = SamplingWorker(sample_freq=self.sampling_freq, max_pressure=self.max_pressure)
        self.sampler.freq_signal.connect(self.on_sampler_freq)
        self.sampler.start()

    # -------- Cavro worker slots --------
    def on_plunger_update(self, Pump_id:int, raw_pos:float):
        # update Pump_data in a thread-safe way
        pos_mm = - raw_pos / 3000.0 * STROKE_MM
        with Pump_data_lock:
            Pump_data[Pump_id]['position'] = pos_mm

    def on_valve_update(self, Pump_id:int, valve_p:int):
        with Pump_data_lock:
            Pump_data[Pump_id]['valve'] = valve_p
        self.panels[Pump_id].update_valve_position(valve_p)

    # -------- UI callbacks --------
    def set_pressure_max_func(self):
        try:
            self.max_pressure = int(self.entry_max_pressure.text())
            self.sampler.set_max_pressure(self.max_pressure)
            print(f"Max pressure set to {self.max_pressure} mbar")
        except ValueError:
            print("Invalid max pressure value. Please enter an integer.")
            return
        

    def btn_zero_all_p_func(self):
        global pressureOffsets
        # Recompute offsets from current Pump_data pressures
        # Map sensor readings to Pump indices using SENSOR_MAP
        with Pump_data_lock:
            for sensorInfo in sensorInfoArray:
                try:
                    sensor_id = str(sensorInfo.indexID)
                    if sensor_id in SENSOR_MAP:
                        idx = SENSOR_MAP[sensor_id]
                        if idx < NUM_PUMPS:
                            pressureOffsets[idx] = pressureOffsets[idx] + Pump_data[idx]['pressure']
                except Exception as e:
                    print(f"Error zeroing sensor {sensorInfo.index}: {e}")
        print("Zeroed pressures:", pressureOffsets)

    def btn_stop_all_func(self):
        message = f"/_T,"
        send_message(None, message, RAW=True)

    def btn_loop_all_func(self):
        try:
            self.max_pressure = int(self.entry_max_pressure.text())

            self.start_pos = int(w.entry_start.text())
            self.start_pos = int(w.start_pos / STROKE_MM * STROKE_INCREMENTS)
            self.stop_pos = int(w.entry_stop.text())
            self.stop_pos = int(w.stop_pos / STROKE_MM * STROKE_INCREMENTS)

            self.loop_cycles = int(self.entry_cycles.text())
            self.max_restarts = int(self.entry_max_restarts.text())
        except ValueError:
            print("Invalid pressure/pos/cycles value. Please enter integers.")
            return
        #message = f"/_Y,g,M500,E,M500,A{self.start_pos},M500,I,M500,A{self.stop_pos},G{self.loop_cycles},M500,E,M500,A{self.start_pos}R"
        message = "".join(
            f"/{ADDRESS[i]}Y,g,M500,E,M500,A{self.start_pos},M500,I,M500,A{self.stop_pos},G{self.loop_cycles},M500,E,M500,A{self.start_pos}R,"
            for i in range(NUM_PUMPS)
        )
        send_message(None, message, RAW=True)

    def btn_set_freq_func(self):
        try:
            freq_local = float(self.entry_set_freq.text())
            if freq_local <= 0:
                raise ValueError()
            self.sampling_freq = freq_local
            # tell sampler to change rate
            self.sampler.set_rate(self.sampling_freq)
            print(f"Sampling freq set to {self.sampling_freq} Hz")
        except Exception:
            print("Invalid frequency value. Please enter a positive number.")

    # ---------------- UI UPDATES ----------------
    def on_sampler_freq(self, hz:float):
        # update displayed frequency (smoothed display could be added)
        self.frequency.setText(f"  {hz:.1f} Hz")

    def update_all_panels(self):
        # Read only the latest sample from data_queue (if present)
        try:
            pressures, positions, valves, timestamp = data_queue.get_nowait()
        except queue.Empty:
            # no new sample; just refresh status from Pump_data
            pressures = None
            positions = None
            valves = None
            timestamp = datetime.now()

        if pressures is not None:
            # push sampled points into each panel's internal buffers
            for i, panel in enumerate(self.panels):
                panel.add_data_point(pressures[i], positions[i], timestamp)

        # always refresh GUI at lower rate
        for panel in self.panels:
            panel.update_plot()

    def _line(self, vertical=False):
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.Shape.VLine if vertical else QtWidgets.QFrame.Shape.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        return line

    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            if hasattr(self, 'sampler'):
                self.sampler.stop()
            if hasattr(self, 'cavro_worker'):
                self.cavro_worker.stop()
        finally:
            event.accept()

# ------------------ SEND MESSAGE ------------------
def send_message(m_id: int, cmd: str, RAW: bool=False) -> None:
    # Build full command with trailing CR and enqueue for cavro_worker to execute
    if RAW:
        full_cmd = cmd if cmd.endswith('\r') else cmd + '\r'
    else:
        full_cmd = f"/{ADDRESS[m_id]}{cmd}\r"
    cavro_cmd_queue.put(full_cmd)

# ------------------ MAIN ------------------
if __name__ == "__main__":
    print("===================== START =====================")
    print("Detected Sensors:", detected_sensors)
    print("Mapped Sensors:", SENSOR_MAP)
    print("Starting GUI...")

    # Initialize Cavro Pumps (safe; enqueued)
    if ser_cavro.is_open:
        try:
            message = "".join(
                f"/{ADDRESS[i]}YR,"
                for i in range(NUM_PUMPS)
            )
            send_message(None, message, RAW=True)
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
            if hasattr(w, 'sampler'):
                w.sampler.stop()
        except Exception:
            pass
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
