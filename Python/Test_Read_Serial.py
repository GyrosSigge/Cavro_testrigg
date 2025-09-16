import serial

ser = serial.Serial('COM4', 115200)

print("Reading from serial port...")

while True:
    line = ser.readline().decode().strip()
    values = line.split(",")
    currents = list(map(float, values[0:5]))
    positions = list(map(float, values[5:10]))
    print(currents, positions)
