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
K_v = Symbol('K_v') # Motor back-emf constant, RPM/V
N_P = Symbol('N_P') # Number of magnetic pole pairs
J   = Symbol('J')   # Rotor inertia
T_l_pnoise = Symbol('T_l_pnoise') # Load torque process noise
i_pnoise = Symbol('i_pnoise') # Current process noise
omega_pnoise = Symbol('omega_pnoise')
theta_pnoise = Symbol('theta_pnoise')
K_t = 30./(K_v*pi) # Motor torque constant.
lambda_r =  2./3. * K_t/N_P # Rotor flux linkage - Ohm, section III, eqn 3.7

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

i_d_dot = (L_q*omega_e_est*i_q_est - R_s*i_d_est + u_d)/L_d
i_q_dot = (-L_d*omega_e_est*i_d_est - R_s*i_q_est - lambda_r*omega_e_est + u_q)/L_q

f = Matrix([
    [omega_r_est + dt*(i_q_est*K_t/J + (L_d-L_q)*i_q_est*i_d_est - T_l_est/J)],
    [theta_e_est + dt*omega_e_est],
    [i_d_est + dt*i_d_dot],
    [i_q_est + dt*i_q_dot],
    [T_l_est]
    ])
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

# P_p: covariance matrix at time k+1
P_p = F*P*F.T + Q
assert P_p.shape == P.shape

# h: predicted measurement
h = zeros(2,1)
h[0:2,0] = R_dq_ab(theta_e_est) * Matrix([i_d_est, i_q_est])

# z: observation
z = toVec(i_ab_m)

# R: observation covariance
R = diag(i_noise**2,i_noise**2) # Covariance of observation vector

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

NIS = (y.T*S_I*y).xreplace(dict(zip(x,x_p)+zip(P,P_p))) # normalized innovation squared

x_n = (x + K*y).xreplace(dict(zip(x,x_p)+zip(P,P_p)))
P_n = ((I-K*H)*P).xreplace(dict(zip(x,x_p)+zip(P,P_p)))
P_n = upperTriangularToVec(P_n)

# Generate code
x_n,P_n,y,NIS,subx = extractSubexpressions([x_n,P_n,y,NIS],'subx',threshold=5)

init_P = upperTriangularToVec(diag(0., math.pi**2, 0., 0., 0.**2))

sys.stdout.write(
'#pragma once\n'
'#include <math.h>\n'
'#include "math_helpers.h"\n'
'\n'
'struct ekf_state_s {\n'
'    float x[5];\n'
'    float P[15];\n'
'    float innov[2];\n'
'    float NIS;\n'
'};\n'
'\n'
'static struct ekf_state_s ekf_state[2];\n'
'static uint8_t ekf_idx = 0;\n'
'\n'
'static void ekf_init(float init_theta) {\n'
'    float* state = ekf_state[ekf_idx].x;\n'
'    float* cov = ekf_state[ekf_idx].P;\n'
)

for i in range(len(init_P)):
    sys.stdout.write('    cov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(init_P[i])))

sys.stdout.write(
'    memset(&ekf_state[ekf_idx], 0, sizeof(ekf_state[ekf_idx]));\n'
'    state[1] = init_theta;\n'
'}\n'
'\n'
'static void ekf_update(float dt, float u_alpha, float u_beta, float i_alpha_m, float i_beta_m) {\n'
'    uint8_t next_ekf_idx = (ekf_idx+1)%2;\n'
'    float* state = ekf_state[ekf_idx].x;\n'
'    float* cov = ekf_state[ekf_idx].P;\n'
'    float* state_n = ekf_state[next_ekf_idx].x;\n'
'    float* cov_n = ekf_state[next_ekf_idx].P;\n'
'    float* innov = ekf_state[next_ekf_idx].innov;\n'
'    float* NIS = &ekf_state[next_ekf_idx].NIS;\n'
'\n'
)

sys.stdout.write('    // %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(subx),))
sys.stdout.write('    static float subx[%u];\n' % (len(subx),))
for i in range(len(subx)):
    sys.stdout.write('    %s = %s;\n' % (subx[i][0], CCodePrinter_float().doprint(subx[i][1])))

for i in range(len(x_n)):
    sys.stdout.write('    state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

for i in range(len(P_n)):
    sys.stdout.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))

for i in range(len(y)):
    sys.stdout.write('    innov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(y[i])))

sys.stdout.write('    *NIS = %s;\n' % (CCodePrinter_float().doprint(NIS[0]),))

sys.stdout.write(
'\n'
'    state_n[1] = wrap_2pi(state_n[1]);\n'
'    ekf_idx = next_ekf_idx;\n'
'}\n'
)





