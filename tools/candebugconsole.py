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

omega_est = 0
omega_meas = 0
theta_est = 0
theta_meas = 0
id_est = 0
id_meas = 0
iq_est = 0
iq_meas = 0
prev_seq1 = 0
prev_seq2 = 0
prev_seq3 = 0

with serial.Serial(args.port[0], int(args.baudrate[0])) as ser:
    try_forever(ser, 'C')
    try_forever(ser, 'S8')
    try_forever(ser, 'O')
    try_forever(ser, 'U921600')
    ser.baudrate = 921600

    while True:
        line = retreive_line(ser)
        #sys.stdout.write('\n'+line+'\n')

        if len(line) == len('t03580000C0FF1B5429C1') and line[0] == 't':
            msgid = struct.unpack('>H', ('0'+line[1:4]).decode('hex'))[0]
            float1, float2 = struct.unpack('<ff', line[5:21].decode('hex'))

            if (msgid&0xff00)>>8 == 0:
                seq = msgid&0xff
                if prev_seq1 != (seq-1) & 0xff:
                    print 0, prev_seq1, seq
                prev_seq1 = seq

            if (msgid&0xff00)>>8 == 1:
                seq = msgid&0xff
                if prev_seq2 != (seq-1) & 0xff:
                    print 1, prev_seq2, seq
                prev_seq2 = seq

            if (msgid&0xff00)>>8 == 2:
                seq = msgid&0xff
                if prev_seq3 != (seq-1) & 0xff:
                    print 2, prev_seq3, seq
                prev_seq3 = seq

            #if msgid == 50:
                #omega_est = float1
                #omega_meas = float2
            #elif msgid == 51:
                #theta_est = float1
                #theta_meas = float2
            #elif msgid == 52:
                #id_est = float1
                #id_meas = float2
            #elif msgid == 53:
                #iq_est = float1
                #iq_meas = float2
                #print "\n %15s %15s %15s %15s\n %15f %15f %15f %15f\n %15f %15f %15f %15f" % ('omega', 'theta', 'id', 'iq', omega_est, theta_est, id_est, iq_est, omega_meas, theta_meas, id_meas, iq_meas)


        #if line[0:4] == 't7FF':
            #msglen = struct.unpack('B', ('0'+line[4]).decode('hex'))
            #sys.stdout.write(struct.unpack('<%us' % (msglen),line[5:].decode('hex'))[0])
