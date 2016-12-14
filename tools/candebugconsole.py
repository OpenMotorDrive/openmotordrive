import serial
import argparse
import struct
import math
import sys

def try_forever(ser, cmd):
    while True:
        ser.write(cmd+'\r\n')
        ret = ser.read(1)
        if ret == '\r':
            return

def retreive_line(ser):
    line = ''
    while True:
        byte = ser.read(1)
        if byte == '\r':
            return line
        else:
            line += byte

parser = argparse.ArgumentParser()
parser.add_argument('--baudrate', nargs=1, dest='baudrate', default=['115200'])
parser.add_argument('port', nargs=1)
args = parser.parse_args()

with serial.Serial(args.port[0], int(args.baudrate[0])) as ser:
    try_forever(ser, 'C')
    try_forever(ser, 'S8')
    try_forever(ser, 'O')
    try_forever(ser, 'U921600')
    ser.baudrate = 921600

    while True:
        line = retreive_line(ser)
        #sys.stdout.write('\n'+line+'\n')
        if line[0:4] == 't7FF':
            msglen = struct.unpack('B', ('0'+line[4]).decode('hex'))
            sys.stdout.write(struct.unpack('<%us' % (msglen),line[5:].decode('hex'))[0])
