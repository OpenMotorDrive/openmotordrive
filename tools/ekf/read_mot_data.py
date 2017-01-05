import csv
import sys
import struct
from math import sqrt

SLIP_END = chr(0xC0)
SLIP_ESC = chr(0xDB)
SLIP_ESC_END = chr(0xDC)
SLIP_ESC_ESC = chr(0xDD)

def slip_decode(in_buf):
    if SLIP_END not in in_buf:
        return (None, in_buf)

    out_buf = ''
    esc_flag = False
    for i in range(len(in_buf)):
        if esc_flag:
            if in_buf[i] == SLIP_ESC_ESC:
                out_buf += SLIP_ESC
            elif in_buf[i] == SLIP_ESC_END:
                out_buf += SLIP_END
            esc_flag = False
        elif in_buf[i] == SLIP_ESC:
            esc_flag = True
        elif in_buf[i] == SLIP_END:
            return (out_buf, in_buf[i+1:])
        else:
            out_buf += in_buf[i]

buf = ''

struct_fmt = '<IfffffffI'

data = {'t_us':[],'dt':[],'theta_e':[], 'omega_e':[], 'i_alpha':[],'i_beta':[],'u_alpha':[],'u_beta':[], 'adc_errcnt':[],'R':0.11,'L':80*1e-6,'Kt':0.02652582384,'J':0.00006}

t0 = None
us_accum = 0

with open(sys.argv[1], 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    reader.next()
    for row in reader:
        buf += row[2][2:].decode('hex')
        frame, buf = slip_decode(buf)
        if frame is not None:
            frame_data = dict(zip(('t_us','dt','theta_e', 'omega_e', 'i_alpha','i_beta','u_alpha','u_beta','adc_errcnt'),struct.unpack(struct_fmt,frame)))
            if t0 is None:
                t0 = frame_data['t_us']
            else:
                us_accum += int(round(frame_data['dt']*1e6))

            print int(round(frame_data['dt']*1e6))
            #print (us_accum - (frame_data['t_us']-t0))/(1e6/(18000/3))

            #frame_data['u_alpha'] *= 1./sqrt(3)
            #frame_data['u_beta'] *= 1./sqrt(3)

            #frame_data['u_alpha'] *= 1./sqrt(2./3.)
            #frame_data['u_beta'] *= 1./sqrt(2./3.)

            #frame_data['i_alpha'] = frame_data['i_alpha'] * (2./3.)/sqrt(2./3.) # for non-power-invariant transform
            #frame_data['i_beta'] = frame_data['i_beta'] * (2./3.)/sqrt(2./3.)


            for key,val in frame_data.iteritems():
                data[key].append(val)

from scipy.io import savemat
savemat('mot_data.mat',data)
