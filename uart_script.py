import serial
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(f"pico says: {line}")