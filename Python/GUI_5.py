import tkinter as tk
from tkinter import ttk
import threading
import time
from datetime import datetime
import serial
from collections import deque

from Fluigent.SDK import fgt_init, fgt_close
from Fluigent.SDK import fgt_get_sensorChannelsInfo
from Fluigent.SDK import fgt_get_sensorUnit, fgt_get_sensorValue

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.dates as mdates
from PIL import Image, ImageTk

stop_event = threading.Event()

DEBUG = False

# --------- Logging SVC FIle ----------
import csv
LOG_FILENAME = f"motor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
log_file = open(LOG_FILENAME, mode='w', newline='')
log_writer = csv.writer(log_file)

# Write header
header = ['Timestamp']
for i in range(1, 6):  # 5 motors
    header += [f'Motor{i}_Pressure', f'Motor{i}_Current', f'Motor{i}_Position']
log_writer.writerow(header)
log_file.flush()


# --------- Pressure Sensor GUI map ---------
# Predefined mapping of SensorID -> motor_data index
sensor_map = {
    "1387328640": 0,  # Motor slot 1
    "1387328640": 1,  # Motor slot 2
    "1387328640": 2,  # Motor slot 2
    "1387328640": 3,  # Motor slot 2
    "1387328640": 4,  # Motor slot 2
    #"1387185280": 1,  # Motor slot 2
    # Add more SensorIDs here as needed
}
## Initialize the Fluigent session
# This step is optional, if not called session will be automatically created
fgt_init()

## Get information about sensors and read them, printing to console
sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()

# --------- SERIAL SETUP CAVRO ----------
SERIAL_PORT = "COM3"
SERIAL_BAUD = 9600
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=SERIAL_BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

# --------- SERIAL SETUP Current Position ----------
ser_ESP32 = serial.Serial('COM4', 115200)
STROKE_MM = 40


# --------- Shared Data ----------
motor_data = [
    {"pressure": 0.0, "current": 0.0, "position": 0},
    {"pressure": 0.0, "current": 0.0, "position": 0},
    {"pressure": 0.0, "current": 0.0, "position": 0},
    {"pressure": 0.0, "current": 0.0, "position": 0},
    {"pressure": 0.0, "current": 0.0, "position": 0},
]

MAX_POINTS = 1000  # keep last N points

# --------- GUI Components ----------
class MotorPanel(ttk.Frame):
    def __init__(self, parent, motor_id, data):
        super().__init__(parent, padding=6)
        self.data = data
        self.pressure = tk.StringVar()
        self.current = tk.StringVar()
        self.position = tk.IntVar()
        self.position.trace_add("write", self._update_position_bar)
        self.motor_id = motor_id

        self.pressure_recent_max = tk.StringVar()
        self.pressure_recent_min = tk.StringVar()
        self.current_recent_max = tk.StringVar()
        self.position_recent_max = tk.StringVar()

        # Store datapoints for graph
        self.time_points = deque(maxlen=MAX_POINTS)
        self.pressure_points = deque(maxlen=MAX_POINTS)
        self.current_points = deque(maxlen=MAX_POINTS)
        self.position_points = deque(maxlen=MAX_POINTS)

        # Buttons and Displays
        group = ttk.LabelFrame(self, text=f"Motor {motor_id}", padding=6)
        group.grid(row=0, column=0)

        controls = ttk.Frame(group)
        controls.grid(row=0, column=0, pady=0, padx=10)

        #Load an image
        img = Image.open("valve_output.png")  # <-- replace with your file
        img = img.resize((50, 50), Image.LANCZOS)  # resize if needed
        self.photo_valve_output = ImageTk.PhotoImage(img)
        img = Image.open("valve_input.png")
        img = img.resize((50, 50), Image.LANCZOS)
        self.photo_valve_input = ImageTk.PhotoImage(img)
        img = Image.open("valve_extra.png")
        img = img.resize((50, 50), Image.LANCZOS)
        self.photo_valve_extra = ImageTk.PhotoImage(img)
        self.image_label = ttk.Label(controls, image=self.photo_valve_output)
        self.image_label.grid(row=0, column=3, rowspan=2, sticky="n")

        buttons = ttk.Frame(controls)
        buttons.grid(row=0, column=0, rowspan=3, columnspan=2)

        ttk.Button(buttons, text="Init", width=13, command=self.on_init).grid(row=0, column=0, columnspan=1, pady=2, padx=2)
        ttk.Button(buttons, text="Move Up", width=13, command=self.on_move_up).grid(row=1, column=0, columnspan=1, pady=2, padx=2)
        ttk.Button(buttons, text="Move Down", width=13, command=self.on_move_down).grid(row=2, column=0, columnspan=1, pady=2, padx=2)
        ttk.Button(buttons, text="Valve Input", width=13, command=self.valve_set_input).grid(row=0, column=1, columnspan=1, pady=2, padx=2)
        ttk.Button(buttons, text="Valve Output", width=13, command=self.valve_set_output).grid(row=1, column=1, columnspan=1, pady=2, padx=2)
        ttk.Button(buttons, text="Valve Extra", width=13, command=self.valve_set_extra).grid(row=2, column=1, columnspan=1, pady=2, padx=2)

        # --- Position bar ---
        self.bar_canvas = tk.Canvas(controls, width=20, height=82, highlightthickness=1, relief="solid")
        self.bar_canvas.grid(row=2, column=3, rowspan=1, sticky="ns")
        self.bar_outline = self.bar_canvas.create_rectangle(5, 5, 15, 115, outline="black", width=1)
        self.bar_fill = self.bar_canvas.create_rectangle(5, 80, 15, 115, fill="blue")

        indicators = ttk.Frame(controls)
        indicators.grid(row=3, column=0, columnspan=4)

        ttk.Label(indicators, text="Pressure:").grid(row=4, column=0, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.pressure, state="readonly", width=6).grid(row=4, column=1, sticky="w", pady=1, padx=2)
        ttk.Label(indicators, text="Recent Max:").grid(row=4, column=2, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.pressure_recent_max, state="readonly", width=6).grid(row=4, column=3, sticky="w", pady=1, padx=2)

        ttk.Label(indicators, text="Current:").grid(row=5, column=0, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.current, state="readonly", width=6).grid(row=5, column=1, sticky="w", pady=1, padx=2)
        ttk.Label(indicators, text="Recent Max:").grid(row=5, column=2, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.current_recent_max, state="readonly", width=6).grid(row=5, column=3, sticky="w", pady=1, padx=2)

        ttk.Label(indicators, text="Position:").grid(row=6, column=0, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.position, state="readonly", width=6).grid(row=6, column=1, sticky="w", pady=1, padx=2)
        ttk.Label(indicators, text="Recent Max:").grid(row=6, column=2, sticky="e", pady=1, padx=2)
        ttk.Entry(indicators, textvariable=self.position_recent_max, state="readonly", width=6).grid(row=6, column=3, sticky="w", pady=1, padx=2)


        # Graph area (Matplotlib figure)
        graph_frame = ttk.Frame(group)
        graph_frame.grid(row=1, column=0, pady=0)

        fig = Figure(figsize=(2.5, 2.5), dpi=100)  

        # Create 3 subplots in a column
        self.ax_pressure = fig.add_subplot(311)
        self.ax_current = fig.add_subplot(312, sharex=self.ax_pressure)
        self.ax_position = fig.add_subplot(313, sharex=self.ax_pressure)

        # Pressure plot
        self.pressure_line, = self.ax_pressure.plot([], [], label="Pressure (mbar)")
        self.ax_pressure.legend(fontsize=6, loc="upper left")
        self.ax_pressure.tick_params(axis='both', labelsize=6)

        # Current plot
        self.current_line, = self.ax_current.plot([], [], label="Current (mA)", color="orange")
        self.ax_current.legend(fontsize=6, loc="upper left")
        self.ax_current.tick_params(axis='both', labelsize=6)

        # Position plot
        self.position_line, = self.ax_position.plot([], [], label="Position (mm)", color="green")
        self.ax_position.legend(fontsize=6, loc="upper left")
        self.ax_position.tick_params(axis='both', labelsize=6)

        # Format x-axis as time
        for ax in (self.ax_pressure, self.ax_current, self.ax_position):
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            ax.xaxis.set_major_locator(mdates.AutoDateLocator())

        fig.autofmt_xdate(rotation=45)
        fig.subplots_adjust(left=0.2, bottom=0.2, right=0.8, top=0.95)

        # Match Tkinter background
        bg_color = self.winfo_toplevel().cget("bg")
        bg_color = "#%02x%02x%02x" % tuple(c // 256 for c in self.winfo_rgb(bg_color))
        fig.patch.set_facecolor(bg_color)

        self.canvas = FigureCanvasTkAgg(fig, master=graph_frame)
        self.canvas.get_tk_widget().pack()
        self.fig = fig  # keep a reference


        # Display Update
        self.update_display()

    def add_datapoint(self, pressure, current, position, t=None):
        """Add a new data point to the history."""
        t = t or datetime.now()
        self.time_points.append(t)
        self.pressure_points.append(pressure)
        self.current_points.append(current)
        self.position_points.append(position)

    def _update_position_bar(self, *args):
        pos = self.position.get()
        pos = max(0, min(STROKE_MM, pos))  # clamp
        y_min, y_max = 5, 115
        total_height = y_max - y_min
        filled_height = int((pos / STROKE_MM) * total_height)
        new_y = y_max - filled_height
        self.bar_canvas.coords(self.bar_fill, 5, new_y, 15, y_max)


    def update_display(self):
        with data_lock:
            p, c, pos = self.data['pressure'], self.data['current'], self.data['position']
        self.pressure.set(f"{p:.1f}")
        self.current.set(f"{c:.1f}")
        self.position.set(f"{pos:.1f}")

    def update_graph(self):
        with data_lock:
            times = list(self.time_points)
            pressures = list(self.pressure_points)
            currents = list(self.current_points)
            positions = list(self.position_points)

        if times:
            # --- x-limits ---
            if len(times) < MAX_POINTS:
                # before deque is full: fit from first to last
                self.ax_pressure.set_xlim(times[0], times[-1])
            else:
                # once deque is full: fixed window = last MAX_POINTS worth of time
                window_start = times[-MAX_POINTS]
                window_end   = times[-1]
                self.ax_pressure.set_xlim(window_start, window_end)

            # --- y-limits (same as your code) ---
            min_val, max_val = min(pressures), max(pressures)
            self.pressure_recent_max.set(f"{max_val:.1f}")
            new_ymin, new_ymax = min(0, min_val), max_val * 1.2
            self.ax_pressure.set_ylim(new_ymin, new_ymax)

            min_val, max_val = min(currents), max(currents)
            self.current_recent_max.set(f"{max_val:.1f}")
            new_ymin, new_ymax = -0.5, max_val * 1.2
            self.ax_current.set_ylim(new_ymin, new_ymax)

            min_val, max_val = min(positions), max(positions)
            self.position_recent_max.set(f"{max_val:.1f}")
            new_ymin, new_ymax = -0.5, max_val * 1.2
            self.ax_position.set_ylim(new_ymin, new_ymax)

            # --- update plots ---
            self.pressure_line.set_data(times, pressures)
            self.current_line.set_data(times, currents)
            self.position_line.set_data(times, positions)
            self.canvas.draw_idle()




    def on_move_up(self): send_message(self.motor_id, f"A3000R")
    def on_move_down(self): send_message(self.motor_id, f"A1000R")
    def on_init(self): 
        send_message(self.motor_id, f"ZR")
        self.image_label.configure(image=self.photo_valve_output)

    def valve_set_input(self): 
        send_message(self.motor_id, f"IR")
        self.image_label.configure(image=self.photo_valve_input)

    def valve_set_output(self): 
        send_message(self.motor_id, f"OR")
        self.image_label.configure(image=self.photo_valve_output)

    def valve_set_extra(self): 
        send_message(self.motor_id, f"ER")
        self.image_label.configure(image=self.photo_valve_extra)

    def on_custom_action_2(self): pass

    


class MultiMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5-Motor Control Panel")

        container = ttk.Frame(root, padding=8)
        container.grid(row=0, column=0)

        self.panels = []
        for i in range(5):
            panel = MotorPanel(container, motor_id=i + 1, data=motor_data[i])
            panel.grid(row=0, column=i, padx=5, sticky="n")
            self.panels.append(panel)

        self.update_all_display()
        self.update_all_graph()

    def update_all_display(self):
        for panel in self.panels:
            panel.update_display()
        self.root.after(100, self.update_all_display)

    def update_all_graph(self):
        for panel in self.panels:
            panel.update_graph()
        self.root.after(1000, self.update_all_graph)




# Keep a small history of timestamps to calculate frequency
POINT_HISTORY = 100
timestamp_history = deque(maxlen=POINT_HISTORY)
last_print_time = 0
PRINT_INTERVAL = 1.0  # seconds

# --------- Read data thread ----------
def read_sensors():
    global last_print_time

    while not stop_event.is_set():
        # --- Read ESP32 Serial ---
        line = ser_ESP32.readline().decode().strip()
        if not line: 
            continue
        values = line.split(",")
        if len(values) < 10:  # expect at least 10 values (5 currents + 5 positions)
            continue
        currents = list(map(float, values[0:5]))
        positions = list(map(float, values[5:10]))

        # --- Read Fluigent Pressure Sensors ---
        for sensorInfo in sensorInfoArray:
            measurement = fgt_get_sensorValue(sensorInfo.index)
            sensor_id = str(sensorInfo.indexID)
            if sensor_id in sensor_map:
                idx = sensor_map[sensor_id]
                motor_data[idx]['pressure'] = measurement
            else:
                print(f"[WARN] Unknown SensorID {sensor_id}, ignoring...")

        # --- Update MotorPanels and add datapoints ---
        with data_lock:
            for i, panel in enumerate(app.panels): 
                motor_data[i]['current'] = currents[i] * -1
                motor_data[i]['position'] = positions[i] / 1023 * STROKE_MM
                panel.add_datapoint(
                    pressure=motor_data[i]['pressure'],
                    current=motor_data[i]['current'],
                    position=motor_data[i]['position']
                )

        # --- Calculate frequency ---
        now = time.time()
        timestamp_history.append(now)
        if now - last_print_time >= PRINT_INTERVAL and len(timestamp_history) > 1:
            intervals = [t2 - t1 for t1, t2 in zip(timestamp_history, list(timestamp_history)[1:])]
            avg_interval = sum(intervals) / len(intervals)
            freq = 1.0 / avg_interval if avg_interval > 0 else 0
            print(f"Data point frequency: {freq:.2f} Hz")
            last_print_time = now

        # --- Log data to CSV ---
        row = [datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')]
        for i in range(5):
            row += [motor_data[i]['pressure'], motor_data[i]['current'], motor_data[i]['position']]
        log_writer.writerow(row)

        if now - last_print_time >= 5:  # every 5s
            log_file.flush()




# ---------- Send I2C to Cavro ----------
def send_message(m_id, cmd: str) -> None:
    cmd = f"/{m_id+1}" + cmd + "\r"
    ser.write((cmd).encode())
    print(cmd)


# --------- Start Everything ----------
if __name__ == "__main__":
    data_lock = threading.Lock()

    root = tk.Tk()
    app = MultiMotorGUI(root)



    data_lock = threading.Lock()

    read_sensors = threading.Thread(target=read_sensors, name="read_sensors", daemon=True)
    read_sensors.start()

    try:
        root.mainloop()
    finally:
        stop_event.set()
        read_sensors.join()
        ser.close()
        ser_ESP32.close()
        fgt_close()
        log_file.close()
        print("GUI Closed, all threads stopped.")
