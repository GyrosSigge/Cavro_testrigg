import serial
import Fluigent.SDK as fg
import time

# --------- SERIAL SETUP ----------
SERIAL_PORT = "COM3"
SERIAL_BAUD = 38400 # Note that the baudrate has to be defined on cavro usin /U41 for 9600 and U47 for 38400
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=SERIAL_BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,
)

# ---------- Send I2C to Cavro ----------
def send_message(cmd: str) -> None:
    cmd = "/" + cmd + "\r"
    ser.write((cmd).encode())
    print(cmd)
    
print("======================")
print("Cavro Control Test")

def get_plunger_position(m_id: int) -> float:
    try:
        if ser.is_open:
            cmd = f"/{m_id+1}?4\r"
            ser.write(cmd.encode())
            response = ser.read_until().decode().strip()
            print(f"Sent: {cmd.strip()} | Received: {response}")
            if response.startswith("/0"):
                pos_str = response[3:-1]  # Remove prefix
                return float(pos_str)
    except Exception as e:
        print(f"Error getting plunger position from Cavro: {e}")
        ser.close()
    return 0.0

def get_valve_position(m_id: int) -> float:
    try:
        if ser.is_open:
            cmd = f"/{m_id+1}?6\r"
            ser.write(cmd.encode())
            response = ser.read_until().decode().strip()
            print(f"Sent: {cmd.strip()} | Received: {response}")
            if response.startswith("/0`"):
                pos_str = response[3:-1]  # Remove prefix
                return int(pos_str)
        else:
            return 0
    except Exception as e:
        print(f"Error getting valve position from Cavro: {e}")
        ser.close()
    return 0.0


while True:
    cmd = input("Enter Command: ")
    send_message(cmd)
    print(ser.read_until().decode())

    print(get_plunger_position(1))
    print(get_valve_position(1))
    