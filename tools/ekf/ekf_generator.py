#!/usr/bin/python
from sympy import *
from sympy.printing.ccode import *
from math import sqrt, floor, fmod
import math
from helpers import *
import sys

# Refs:
# "Estimation of Rotor Position and Speed for Sensorless DSP-based PMSM Drives," Ciabattoni, et al., 2011
# "Adaptive Extended Kalman Filter for Robust Sensorless Control of PMSM Drives," Ciabattoni, et al., 2011
#     http://www.nt.ntnu.no/users/skoge/prost/proceedings/cdc-ecc-2011/data/papers/1796.pdf
# "Extended Kalman Filter Application in Permanent Magnet Synchronous Motor Sensorless Control," Qiu, 2003
# "Extended Kalman Filter Based Speed Sensorless PMSM Control with Load Reconstruction," Janiszewski, 2010
# "Dynamic Model of PM Synchronous Motors," Ohm, 2000
#     http://www.drivetechinc.com/articles/IM97PM_Rev1forPDF.pdf

# Parameters
dt  = Symbol('dt')  # Time step
R_s = Symbol('R_s') # Stator resistance
L_d = Symbol('L_d')   # L_d
L_q = Symbol('L_q')   # L_q
lambda_r = Symbol('lambda_r') # Rotor flux linkage
N_P = Symbol('N_P') # Number of magnetic pole pairs
J   = Symbol('J')   # Rotor inertia
T_l_pnoise = Symbol('T_l_pnoise') # Load torque process noise
i_pnoise = Symbol('i_pnoise') # Current process noise
omega_pnoise = Symbol('omega_pnoise')
theta_pnoise = Symbol('theta_pnoise')
i_delay = Symbol('i_delay')

i0 = Symbol('i0')
a = Symbol('a')
b = Symbol('b')

# Inputs
u_ab = Matrix(symbols('u_alpha u_beta')) # Stator voltages
u_noise = Symbol('u_noise') # Additive noise on stator voltage

# Measurements
i_ab_m = Matrix(symbols('i_alpha_m i_beta_m')) # Stator currents observed
i_noise = Symbol('i_noise') # Additive noise on stator currents

# States
omega_r_est, theta_e_est, i_d_est, i_q_est, T_l_est = symbols('state[0:5]')
x = toVec(omega_r_est, theta_e_est, i_d_est, i_q_est, T_l_est)
nStates = len(x)

# Covariance matrix
P = compressedSymmetricMatrix('cov', nStates)

# Derived variables
omega_e_est = omega_r_est*N_P
theta_r_est = theta_e_est/N_P

R_ab_dq = lambda theta: Matrix([[ cos(theta), sin(theta)],
                                [-sin(theta), cos(theta)]])
R_dq_ab = lambda theta: R_ab_dq(theta).T

# f: state-transtition model
#next_theta_e = theta_e_est + dt*omega_e_est
u_dq = R_ab_dq(theta_e_est) * u_ab
u_d = u_dq[0]
u_q = u_dq[1]

T_output = N_P * (3*(L_d-L_q)*i_d_est + sqrt(6)*lambda_r) * i_q_est / 2
omega_dot = (T_output - T_l_est)/J

i_d_dot = (L_q*omega_e_est*i_q_est - R_s*i_d_est + u_d)/L_d
i_q_dot = (-L_d*omega_e_est*i_d_est - R_s*i_q_est - sqrt(6)/2*lambda_r*omega_e_est + u_q)/L_q

x_dot = Matrix([
    omega_dot,
    omega_e_est,
    i_d_dot,
    i_q_dot,
    0
    ])

f = x+dt*x_dot

assert f.shape == x.shape

# F: linearized state-transition model, AKA "A" in literature
F = f.jacobian(x)

# u: control input vector
u = toVec(u_ab)

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

# w_u_sigma: additive noise on u
w_u_sigma = Matrix([u_noise, u_noise])

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# Q: covariance of additive noise on x
Q = G*Q_u*G.T
Q += diag(omega_pnoise**2, theta_pnoise**2, 0**2, 0**2, T_l_pnoise**2)

x_p = f
x_at_curr_meas = x+(dt-i_delay)*x_dot

# P_p: covariance matrix at time k+1
P_p = F*P*F.T + Q
assert P_p.shape == P.shape
P_p = upperTriangularToVec(P_p)

#x_at_curr_meas = x_at_curr_meas.xreplace(subs)
#x_p = x_p.xreplace(subs)
#P_p = P_p.xreplace(subs)

# h: predicted measurement
h = zeros(2,1)
h[0:2,0] = R_dq_ab(theta_e_est) * Matrix([i_d_est, i_q_est])

# z: observation
z = toVec(i_ab_m)

z_norm = (i_ab_m[0]**2+i_ab_m[1]**2)**0.5

# R: observation covariance
R = diag((i_noise)**2,(i_noise)**2) # Covariance of observation vector

# y: innovation vector
y = z-h

# H: measurement sensitivity matrix
H = h.jacobian(x)

# S: innovation covariance
S = H*P*H.T + R

S_I = quickinv_sym(S)

# K: Kalman gain
K = P*H.T*S_I

I = eye(nStates)

NIS = y.T*S_I*y # normalized innovation squared

x_n = x + K*y
P_n = (I-K*H)*P
P_n = upperTriangularToVec(P_n)

#x_n = x_n.xreplace(subs)
#P_n = P_n.xreplace(subs)
#NIS = NIS.xreplace(subs)

# Generate code
x_p,P_p,x_at_curr_meas,pred_subx = extractSubexpressions([x_p,P_p,x_at_curr_meas],'subx',threshold=5)

x_n,P_n,NIS,fuse_subx = extractSubexpressions([x_n,P_n,NIS],'subx',threshold=5)

init_P = upperTriangularToVec(diag(0., math.pi**2, 0., 0., 0.**2))

sys.stdout.write(
'#pragma once\n'
'#include <math.h>\n'
'#include "math_helpers.h"\n'
'\n'
'#ifndef FTYPE\n'
'  #define FTYPE float\n'
'#endif\n'
'\n'
'struct ekf_state_s {\n'
'    FTYPE x[5];\n'
'    FTYPE P[15];\n'
'    FTYPE innov[2];\n'
'    FTYPE NIS;\n'
'};\n'
'\n'
'static struct ekf_state_s ekf_state[2];\n'
'static uint8_t ekf_idx = 0;\n'
'static FTYPE x_at_curr_meas[5];'
'\n'
'static void ekf_init(FTYPE init_theta) {\n'
'    FTYPE* state = ekf_state[ekf_idx].x;\n'
'    FTYPE* cov = ekf_state[ekf_idx].P;\n'
)

for i in range(len(init_P)):
    sys.stdout.write('    cov[%u] = %s;\n' % (i, CCodePrinter().doprint(init_P[i])))

sys.stdout.write(
'    memset(&ekf_state[ekf_idx], 0, sizeof(ekf_state[ekf_idx]));\n'
'    state[1] = init_theta;\n'
'}\n'
'\n'
)

sys.stdout.write('static FTYPE subx[%u];\n' % (max(len(pred_subx),len(fuse_subx)),))
sys.stdout.write(
'static void ekf_predict(FTYPE dt, FTYPE u_alpha, FTYPE u_beta) {\n'
'    uint8_t next_ekf_idx = (ekf_idx+1)%2;\n'
'    FTYPE* state = ekf_state[ekf_idx].x;\n'
'    FTYPE* cov = ekf_state[ekf_idx].P;\n'
'    FTYPE* state_n = ekf_state[next_ekf_idx].x;\n'
'    FTYPE* cov_n = ekf_state[next_ekf_idx].P;\n'
)

sys.stdout.write('    // %u operations\n' % (count_ops(x_p)+count_ops(P_p)+count_ops(pred_subx),))

for i in range(len(pred_subx)):
    sys.stdout.write('    %s = %s;\n' % (pred_subx[i][0], CCodePrinter().doprint(pred_subx[i][1])))

for i in range(len(x_p)):
    sys.stdout.write('    state_n[%u] = %s;\n' % (i, CCodePrinter().doprint(x_p[i])))

for i in range(len(x_at_curr_meas)):
    sys.stdout.write('    x_at_curr_meas[%u] = %s;\n' % (i, CCodePrinter().doprint(x_at_curr_meas[i])))

for i in range(len(P_p)):
    sys.stdout.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter().doprint(P_p[i])))

sys.stdout.write(
'\n'
'    state_n[1] = wrap_2pi(state_n[1]);\n'
'    ekf_idx = next_ekf_idx;\n'
'}\n'
)

sys.stdout.write(
'\n'
'static void ekf_update(FTYPE i_alpha_m, FTYPE i_beta_m) {\n'
'    uint8_t next_ekf_idx = (ekf_idx+1)%2;\n'
'    FTYPE* state = x_at_curr_meas;\n'
'    FTYPE* cov = ekf_state[ekf_idx].P;\n'
'    FTYPE* state_n = ekf_state[next_ekf_idx].x;\n'
'    FTYPE* cov_n = ekf_state[next_ekf_idx].P;\n'
'    FTYPE* innov = ekf_state[next_ekf_idx].innov;\n'
'    FTYPE* NIS = &ekf_state[next_ekf_idx].NIS;\n'
'\n'
)

sys.stdout.write('    // %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(fuse_subx),))
for i in range(len(fuse_subx)):
    sys.stdout.write('    %s = %s;\n' % (fuse_subx[i][0], CCodePrinter().doprint(fuse_subx[i][1])))

for i in range(len(x_n)):
    sys.stdout.write('    state_n[%u] = %s;\n' % (i, CCodePrinter().doprint(x_n[i])))

for i in range(len(P_n)):
    sys.stdout.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter().doprint(P_n[i])))

for i in range(len(y)):
    sys.stdout.write('    innov[%u] = %s;\n' % (i, CCodePrinter().doprint(y[i])))

sys.stdout.write('    *NIS = %s;\n' % (CCodePrinter().doprint(NIS[0]),))

sys.stdout.write(
'\n'
'    state_n[1] = wrap_2pi(state_n[1]);\n'
'    ekf_idx = next_ekf_idx;\n'
'}\n'
)





