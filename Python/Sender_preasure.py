"""
Enkelt script för att skicka tryckdata från Fluigent sensor till en Raspberry Pi via UDP.

Först skrivs sensorinformation ut till konsolen.
Sedan skickas tryckdata från varje sensor till Raspberry Pi varje sekund.

Om man vill kan man ta bort: try... except KeyboardInterrupt,
den finns bara för att kunna avbryta scriptet med Ctrl+C
och stänga ner socketen på ett snyggt sätt.
"""

import socket
import time
import datetime
from Fluigent.SDK import fgt_init, fgt_close
from Fluigent.SDK import fgt_get_sensorChannelsInfo
from Fluigent.SDK import fgt_get_sensorUnit, fgt_get_sensorRange, fgt_get_sensorValue

## Raspberry Pi's IP address and port for receiving UDP messages
# This should match the settings on the Raspberry Pi
HOST = "192.168.10.2"  # Raspberry Pi's static Ethernet IP
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

## Initialize the Fluigent session
# This step is optional, if not called session will be automatically created
fgt_init()

## Get information about sensors and read them, printing to console
sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()
for i, sensorInfo in enumerate(sensorInfoArray):
    print('Sensor channel info at index: {}'.format(i))
    print(sensorInfo)
    print("Sensor type: {}".format(sensorTypeArray[i]))
    # Get measurement unit
    unit = fgt_get_sensorUnit(i)
    # Get sensor range
    minSensor, maxSensor = fgt_get_sensorRange(i)
    print("Range {:0.2f} to {:0.2f} {}".format(minSensor, maxSensor, unit))

## Read the sensor repeatedly and send data via UDP
print("[PC] Starting to send messages to Raspberry Pi...")
message_delay = 0.1  # Delay between messages in seconds
try:
    while True:
        for i, sensorInfo in enumerate(sensorInfoArray):
            message = ""
            now = datetime.datetime.now()   # Get time
            time_formated = now.strftime('%Y-%m-%d %H:%M:%S.') + str(int(now.microsecond / 100000))   # Format time
            unit = fgt_get_sensorUnit(i)
            measurement = fgt_get_sensorValue(i)
            message = '{}SensorID={}, Pressure={:0.2f}{}, Time={}'.format(message, sensorInfo.indexID, measurement, unit, time_formated)

            print(f"[PC] Sent: {message}")
            sock.sendto(message.encode('utf-8'), (HOST, PORT))  # Send UDP message
            time.sleep(message_delay)  # Send once per second

except KeyboardInterrupt:
    print("\n[PC] Stopped sending")
finally:
    sock.close()
    fgt_close()
