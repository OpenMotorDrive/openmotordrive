import serial
import sys

with open('output_data', 'wb') as f:
    with serial.Serial(sys.argv[1], 4500000) as ser:
        while ord(ser.read()) != 0x55:
            continue

        print "start byte recvd"

        while True:
            f.write(ser.read())
