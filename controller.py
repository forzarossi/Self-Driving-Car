import serial
import time

READPING = 1

# Setup of serial communication
port = "/dev/ttyACM0"
rate = 9600
s1 = serial.Serial(port, rate)
s1.flushInput()

while True:
    PingData = -1
    s1.write(str(READPING))
    print("Requesting IR Data from Pi...")
    if s1.inWaiting() > 0:
        PingData = s1.read(1)
    if PingData != -1:
        print("Ping Data (microseconds)")
        print(PingData)
        print("Ping Data (in)")
        print(PingData / 74 / 2)
        print("Ping Data (cm)")
        print(PingData / 29 / 2)
    time.sleep(2)