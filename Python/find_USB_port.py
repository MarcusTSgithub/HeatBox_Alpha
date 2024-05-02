# Script use to find the name of the USB port used for the Arduino. This output is necessary for the other Python script to run
# and should be plugged in on row 9 at "SERIAL_PORT = '[name of port]'"

from serial.tools import list_ports

ports = list_ports.comports()
for p in ports: 
    print(p)
