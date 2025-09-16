"""
Enkelt script för att skicka tidsstämplar till en Raspberry Pi via UDP.

Om man vill kan man ta bort: try... except KeyboardInterrupt,
den finns bara för att kunna avbryta scriptet med Ctrl+C
och stänga ner socketen på ett snyggt sätt.
"""

import socket
import time
import datetime

HOST = "192.168.10.2"  # Raspberry Pi's static Ethernet IP
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("[PC] Starting to send messages to Raspberry Pi...")
print("[PC] Press Ctrl+C in terminal to stop.")
time.sleep(1)

try:
    while True:
        now = datetime.datetime.now()   # Get time
        message = now.strftime('%Y-%m-%d %H:%M:%S.') + str(int(now.microsecond / 100000))   # Format time

        sock.sendto(message.encode('utf-8'), (HOST, PORT))  # Send UDP message

        print(f"[PC] Sent: {message}")
        time.sleep(1)  # Send once per second
except KeyboardInterrupt:
    print("\n[PC] Stopped sending")
finally:
    sock.close()
