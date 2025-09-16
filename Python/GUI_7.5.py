import pyqtgraph as pg
import sys, queue, threading, time, serial, csv
from PyQt6 import QtWidgets, QtGui, QtCore
from datetime import datetime
from Fluigent.SDK import fgt_init, fgt_close, fgt_get_sensorChannelsInfo, fgt_get_sensorValue

# ------------------ CONFIG ------------------
STROKE_MM = 40
MAX_POINTS = 1000  # max points per plot
NUM_MOTORS = 5

SERIAL_PORT_CAVRO = "COM3"
SERIAL_BAUD_CAVRO = 38400
SERIAL_PORT_ESP32 = "COM4"
SERIAL_BAUD_ESP32 = 115200

LOG_FILENAME = f"motor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

freq = 0.0  # Data frequency

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
pressureOffsets = [0.0]*len(sensorInfoArray)  # To store zeroed pressure offsets

sensor_map = {'1387328640': 0, '1387336832': 1, '1387320448': 2, '1387185280': 3, '1387345024': 4}

# ------------------ SERIAL ------------------
ser_cavro = serial.Serial()
ser_esp32 = serial.Serial()

ser_cavro.port = SERIAL_PORT_CAVRO
ser_cavro.baudrate = SERIAL_BAUD_CAVRO
ser_cavro.timeout = 1
ser_esp32.port = SERIAL_PORT_ESP32
ser_esp32.baudrate = SERIAL_BAUD_ESP32
ser_esp32.timeout = 1

try:
    ser_cavro.open()
    ser_esp32.open()
except Exception as e:
    print(f"Error opening serial ports: {e}")

# ------------------ MOTOR DATA ------------------
motor_data = [{"pressure":0.0,"current":0.0,"position":0.0} for _ in range(NUM_MOTORS)]

# ------------------ TIME AXIS ------------------
class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        """Convert float timestamps to HH:MM:SS"""
        return [datetime.fromtimestamp(value).strftime("%H:%M:%S") for value in values]

# ------------------ CAVRO SERIAL WORKER (NON-BLOCKING GUI) ------------------
class SerialCavroWorker(QtCore.QThread):
    plunger_signal = QtCore.pyqtSignal(int, float)  # motor_id, plunger_pos
    valve_signal   = QtCore.pyqtSignal(int, int)    # motor_id, valve_pos

    def __init__(self, num_motors:int, parent=None):
        super().__init__(parent)
        self.num_motors = num_motors
        self._running = True

    def run(self):
        while self._running:
            try:
                if ser_cavro.is_open:
                    for motor_id in range(self.num_motors):
                        # --- Plunger position ---
                        try:
                            cmd = f"/{motor_id+2}?\r"

                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode().strip()
                            if resp.startswith("/0"):
                                pos = float(resp[3:-1])
                                self.plunger_signal.emit(motor_id, pos)
                        except Exception as e:
                            # keep the loop alive even if one read fails
                            print("Reacieved message:", resp)
                            print(f"Cavro plunger read error (motor {motor_id}): {e}")

                        # --- Valve position ---
                        try:
                            cmd = f"/{motor_id+2}?6\r"

                            ser_cavro.write(cmd.encode())
                            resp = ser_cavro.read_until().decode().strip()
                            if resp.startswith("/0"):
                                val = int(resp[3:-1])
                                self.valve_signal.emit(motor_id, val)
                        except Exception as e:
                            print(f"Cavro valve read error (motor {motor_id}): {e}")
            except Exception as e:
                print(f"SerialCavroWorker error: {e}")

            self.msleep(100)  # throttle polling

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
        self.btn_valve_in.clicked.connect(self.btn_valve_in_func)
        self.btn_valve_out.clicked.connect(self.btn_valve_out_func)
        self.btn_valve_ext.clicked.connect(self.btn_valve_ext_func)
        self.btn_loop.clicked.connect(self.btn_loop_func)
        self.btn_stop.clicked.connect(self.btn_stop_func)

        # --- Status layout: labels on left, image + bar on right ---
        status_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(status_layout)

        # Labels (left)
        labels_layout = QtWidgets.QVBoxLayout()
        self.status_pressure = QtWidgets.QLabel("Pressure: 0.0")
        self.status_current  = QtWidgets.QLabel("Current: 0.0")
        self.status_position = QtWidgets.QLabel("Position: 0.0")
        labels_layout.addWidget(self.status_pressure)
        labels_layout.addWidget(self.status_current)
        labels_layout.addWidget(self.status_position)
        labels_layout.addStretch()
        status_layout.addLayout(labels_layout)

        # Image + vertical bar (right)
        img_bar_layout = QtWidgets.QVBoxLayout()

        # Image
        self.image_label = QtWidgets.QLabel()
        self.set_image("valve_output.png")  # default image
        img_bar_layout.addWidget(self.image_label, alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

        # Vertical position bar
        self.position_bar = QtWidgets.QProgressBar()
        self.position_bar.setOrientation(QtCore.Qt.Orientation.Vertical)
        self.position_bar.setRange(0, STROKE_MM-10)
        self.position_bar.setValue(STROKE_MM-10)
        self.position_bar.setTextVisible(False)
        self.position_bar.setFixedHeight(30)
        img_bar_layout.addWidget(self.position_bar, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        status_layout.addLayout(img_bar_layout)

        # Plots (your existing code)
        self.plot_pressure = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Pressure (mbar)")
        self.plot_current  = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Current (mA)")
        self.plot_position = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')}, title="Position (mm)")

        main_layout.addWidget(self.plot_pressure)
        main_layout.addWidget(self.plot_current)
        main_layout.addWidget(self.plot_position)

        self.curve_pressure = self.plot_pressure.plot(pen='r')
        self.curve_current  = self.plot_current.plot(pen='g')
        self.curve_position = self.plot_position.plot(pen='b')

        # Data storage
        self.timestamps = []
        self.pressures = []
        self.currents  = []
        self.positions = []

    # -------------------- IMAGE UPDATE --------------------
    def set_image(self, filename):
        pixmap = QtGui.QPixmap(filename).scaled(
            50, 50,
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation
        )
        pixmap.setDevicePixelRatio(self.devicePixelRatio())
        self.image_label.setPixmap(pixmap)

    # -------------------- BUTTON CALLBACKS --------------------
    def btn_init_func(self):
        send_message(self.motor_id, "ZR")

    def btn_valve_in_func(self):
        send_message(self.motor_id, "IR")

    def btn_valve_out_func(self):
        send_message(self.motor_id, "OR")

    def btn_valve_ext_func(self):
        send_message(self.motor_id, "ER")

    def btn_loop_func(self):
        try:
            w.max_pressure = int(w.entry_max_pressure.text())
            w.start_pos = int(w.entry_start.text())
            w.stop_pos = int(w.entry_stop.text())
            w.loop_cycles = int(w.entry_cycles.text())
        except ValueError:
            print("Invalid pressure/pos/cycles value. Please enter integers.")
            return
        message = f"Z,g,M500,E,M500,A{w.start_pos},M500,I,M500,A{w.stop_pos},G{w.loop_cycles},M500,E,M500,A{w.start_pos}R,"
        send_message(self.motor_id, message)

    def btn_stop_func(self):
        send_message(self.motor_id, "T")        

    # -------------------- DATA & PLOTS --------------------
    def add_data_point(self, p, c, pos, timestamp):
        self.timestamps.append(timestamp.timestamp())
        self.pressures.append(p)
        self.currents.append(c)
        self.positions.append(pos)
        self.timestamps = self.timestamps[-MAX_POINTS:]
        self.pressures = self.pressures[-MAX_POINTS:]
        self.currents = self.currents[-MAX_POINTS:]
        self.positions = self.positions[-MAX_POINTS:]
        motor_data[self.motor_id]['pressure'] = p
        motor_data[self.motor_id]['current'] = c
        motor_data[self.motor_id]['position'] = pos

    def update_plot(self):
        if self.timestamps:
            self.curve_pressure.setData(self.timestamps, self.pressures)
            self.curve_current.setData(self.timestamps, self.currents)
            self.curve_position.setData(self.timestamps, self.positions)
            self.plot_pressure.setYRange(min(min(self.pressures), -1), max(max(self.pressures), 1))
            self.plot_current.setYRange(0, max(max(self.currents), 1))
            self.plot_position.setYRange(min(min(self.positions), -1), 0)

        self.status_pressure.setText(f"Pressure: {motor_data[self.motor_id]['pressure']:.1f}")
        self.status_current.setText(f"Current: {motor_data[self.motor_id]['current']:.1f}")
        self.status_position.setText(f"Position: {motor_data[self.motor_id]['position']:.1f}")
        self.position_bar.setValue(int(STROKE_MM + (motor_data[self.motor_id]['position'])))

    # -------------------- ASYNC UPDATERS FROM WORKER --------------------
    def update_plunger_position(self, pos: float):
        self.plunger_position = pos
        # If you want to visualize Cavro plunger pos instead of ESP32 pos, uncomment next line
        # motor_data[self.motor_id]['position'] = - pos / 3000 * STROKE_MM
        # pos = - pos / 3000 * STROKE_MM
        # print(f"Motor {self.motor_id+1} plunger pos: {pos}")

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

        # Main vertical layout: [controls on top], [line], [panels row]
        main_vlayout = QtWidgets.QVBoxLayout(central_widget)
        main_vlayout.setContentsMargins(8, 8, 8, 8)
        main_vlayout.setSpacing(8)

        # ================= CONTROL AREA =================
        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setSpacing(12)
        main_vlayout.addLayout(controls_layout)

        # ===== Actions =====
        action_box = QtWidgets.QGroupBox("Actions")
        action_layout = QtWidgets.QFormLayout(action_box)
        self.btn_loop_all   = QtWidgets.QPushButton("Start Loop All")
        self.btn_zero_all_p = QtWidgets.QPushButton("Zero All Pressures")
        action_layout.addWidget(self.btn_loop_all)
        action_layout.addWidget(self.btn_zero_all_p)

        self.btn_stop_all = QtWidgets.QPushButton("STOP ALL")
        self.btn_stop_all.setStyleSheet("background-color: red; color: white; font-weight: bold")
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
        settings_box = QtWidgets.QGroupBox("Settings")
        settings_layout = QtWidgets.QGridLayout(settings_box)

        self.entry_max_pressure = QtWidgets.QLineEdit("7000")
        self.btn_set_pressure_max = QtWidgets.QPushButton("Apply")
        settings_layout.addWidget(QtWidgets.QLabel("Max/Stop"))
        settings_layout.addWidget(self.entry_max_pressure, 0, 1)
        settings_layout.addWidget(self.btn_set_pressure_max, 0, 2)

        self.entry_set_freq = QtWidgets.QLineEdit("100")
        self.btn_set_freq = QtWidgets.QPushButton("Apply")
        settings_layout.addWidget(QtWidgets.QLabel("Data Sample Freq:"), 1, 0)
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
        self.btn_zero_all_p.clicked.connect(lambda: self.btn_zero_all_p_func())
        self.btn_stop_all.clicked.connect(lambda: self.btn_stop_all_func()) 
        self.btn_loop_all.clicked.connect(lambda: self.btn_loop_all_func()) 
        self.btn_set_pressure_max.clicked.connect(lambda: self.set_pressure_max_func()) 
        self.btn_set_freq.clicked.connect(lambda: self.btn_set_freq_func())

        # Initialize parameters
        self.max_pressure = int(self.entry_max_pressure.text())
        self.start_pos = int(self.entry_start.text())
        self.stop_pos = int(self.entry_stop.text())
        self.loop_cycles = int(self.entry_cycles.text())

        # ================= BOTTOM AREA =================
        # Top line spanning full width
        top_line = self._line(vertical=False)
        top_line.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                               QtWidgets.QSizePolicy.Policy.Fixed)
        main_vlayout.addWidget(top_line)

        # Row with all motor panels
        panels_row = QtWidgets.QHBoxLayout()
        panels_row.setSpacing(8)
        main_vlayout.addLayout(panels_row)

        self.panels = [MotorPanel(i) for i in range(NUM_MOTORS)]
        for panel in self.panels:
            panels_row.addWidget(panel)
            panel.update_plot()  # force initial update

        # ================= TIMERS =================
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_all_panels)
        self.update_timer.start(50)  # 20 Hz

        self.serial_start = QtCore.QTimer(self)
        self.serial_start.timeout.connect(self.open_serial)
        self.serial_start.start(1000)  # 1 Hz

        # ================= CAVRO WORKER =================
        self.cavro_worker = SerialCavroWorker(NUM_MOTORS)
        self.cavro_worker.plunger_signal.connect(self.on_plunger_update)
        self.cavro_worker.valve_signal.connect(self.on_valve_update)
        self.cavro_worker.start()

    # -------- Cavro worker slots --------
    def on_plunger_update(self, motor_id:int, pos:float):
        # Propagate to panel (and optionally override motor_data if desired)
        self.panels[motor_id].update_plunger_position(pos)

    def on_valve_update(self, motor_id:int, valve_p:int):
        self.panels[motor_id].update_valve_position(valve_p)

    def set_pressure_max_func(self):
        try:
            self.max_pressure = int(self.entry_max_pressure.text())
        except ValueError:
            print("Invalid max pressure value. Please enter an integer.")
            return
        print(f"Max pressure set to {self.max_pressure} mbar")

    def btn_zero_all_p_func(self):
        global pressureOffsets
        pressureOffsets = [pressureOffsets[i] + motor_data[i]['pressure'] for i in range(len(sensorInfoArray))]
        print("Zeroed pressures:", pressureOffsets)

    def btn_stop_all_func(self):
        message = ""
        for i in range(NUM_MOTORS):
            message += f"/{i+2}T,"
        send_message(None, message, RAW=True)

    def btn_set_freq_func(self):
        if ser_esp32.is_open:
            try:
                freq_local = int(self.entry_set_freq.text())
                msg = f"SETFREQ {freq_local}\r"

                ser_esp32.write(msg.encode())
                print(msg)
            except ValueError:
                print("Invalid frequency value. Please enter an integer.")
                return
        
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
            message += f"/{i+2}Z,g,M500,O,M500,A{self.start_pos},M500,I,M500,A{self.stop_pos},G{self.loop_cycles},M500,E,M500,A{self.start_pos}R,"
        send_message(None, message, RAW=True)

    def open_serial(self):
        try:
            if not ser_cavro.is_open:
                ser_cavro.open()
        except Exception as e:
            print(f"Error opening serial ports Cavro: {e}")

        try:
            if not ser_esp32.is_open:
                ser_esp32.open()
        except Exception as e:
            print(f"Error opening serial ports ESP32: {e}")

    def update_all_panels(self):
        """Pull all available queue data and update panels in sync"""
        try:
            while True:
                pressures, currents, positions, timestamp = data_queue.get_nowait()
                for i, panel in enumerate(self.panels):
                    panel.add_data_point(pressures[i], currents[i], positions[i], timestamp)

                    if pressures[i] >= self.max_pressure:
                        send_message(i, "T")
        except queue.Empty:
            pass

        for panel in self.panels:
            panel.update_plot()

        self.frequency.setText(f"  {freq:.1f} Hz")

    def _line(self, vertical=False):
        line = QtWidgets.QFrame()
        line.setFrameShape(
            QtWidgets.QFrame.Shape.VLine if vertical else QtWidgets.QFrame.Shape.HLine
        )
        line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        return line

    def closeEvent(self, event: QtGui.QCloseEvent):
        # Ensure worker is stopped cleanly
        try:
            if hasattr(self, 'cavro_worker'):
                self.cavro_worker.stop()
        finally:
            event.accept()

# ------------------ SERIAL / SENSOR THREAD (ESP32 + Fluigent) ------------------
def read_sensors():
    last_print_time = time.time()
    line_count = 0  # count processed lines
    while not stop_event.is_set():
        try:
            if ser_esp32.is_open:
                line = ser_esp32.readline().decode().strip()
                if not line:
                    continue
                values = line.split(",")
                if len(values) < NUM_MOTORS*2:
                    continue

                currents = list(map(float, values[:NUM_MOTORS]))
                currents = [-i for i in currents]
                positions = list(map(float, values[NUM_MOTORS:NUM_MOTORS*2]))   # Warning, hard coded
                positions = [-i/1023*STROKE_MM for i in positions]

                # Fluigent pressures
                pressures_temp = [0.0]*len(sensorInfoArray)
                for sensorInfo in sensorInfoArray:
                    measurement = fgt_get_sensorValue(sensorInfo.index)
                    sensor_id = str(sensorInfo.indexID)
                    if sensor_id in sensor_map:
                        idx = sensor_map[sensor_id]
                        pressures_temp[idx] = measurement - pressureOffsets[idx]

                pressures = pressures_temp[:NUM_MOTORS]  # Only take first NUM_MOTORS sensors 

                # Enqueue once for all panels
                data_queue.put((pressures, currents, positions, datetime.now()))

                # Log CSV
                row = [datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')]
                for i in range(NUM_MOTORS):
                    row += [pressures[i], currents[i], positions[i]]
                log_writer.writerow(row)

                # Count for frequency calculation
                line_count += 1

                # Flush & print frequency periodically
                now = time.time()
                if now - last_print_time >= 2:
                    global freq
                    freq = line_count / (now - last_print_time)
                    log_file.flush()
                    line_count = 0
                    last_print_time = now   

        except Exception as e:
            print(f"Error reading sensors from ESP32: {e}")
            try:
                ser_esp32.close()
            except Exception:
                pass
            time.sleep(1)

# ------------------ SEND MESSAGE ------------------
def send_message(m_id: int, cmd: str, RAW: bool=False) -> None:
    try:
        if ser_cavro.is_open:
            if RAW:
                full_cmd = f"{cmd}\r"

                ser_cavro.write(full_cmd.encode())
                print(full_cmd)
            else:
                full_cmd = f"/{m_id+2}{cmd}\r"

                ser_cavro.write(full_cmd.encode())
                print(full_cmd)
    except Exception as e:
        print(f"Error sending message to Cavro: {e}")
        try:
            ser_cavro.close()
        except Exception:
            pass

# ------------------ MAIN ------------------
if __name__ == "__main__":
    print("===================== START =====================")
    print("Detected Sensors:", detected_sensors)
    print("Mapped Sensors:", sensor_map)
    print("Starting GUI...")

    if ser_esp32.is_open:
        try:
            freq = 100
            msg = f"SETFREQ {freq}\r"

            ser_esp32.write(msg.encode())
            print(msg)
        except ValueError:
            print("Invalid frequency value. Please enter an integer.")
        
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
            ser_esp32.close()
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