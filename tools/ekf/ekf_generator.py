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

def rk(a,b,c,x_0,x_dot,dt):
    N = a.rows
    assert a.cols == N and len(b) == N and len(c) == N

    k = []

    for i in range(N):
        x_n = x_0
        for j in range(1,i):
            x_n += dt*a[i,j]*k[j]
        k.append(x_dot.xreplace(dict(zip(x_0, x_n))))

    x_n = x_0
    for i in range(N):
        x_n += dt*b[i]*k[i]

    return x_n

def rk3(x_0, x_dot, dt):
    a = Matrix([[0, 0, 0],
                [Rational(1,2),0,0],
                [-1,2,0]])
    b = Matrix([Rational(1,6), Rational(2,3), Rational(1,6)])
    c = Matrix([0, Rational(1,2),1])

    return rk(a,b,c,x_0,x_dot,dt)

# Parameters
dt  = Symbol('dt')  # Time step
R_s = Symbol('this->params.R_s') # Stator resistance
L_d = Symbol('this->params.L_d')   # L_d
L_q = Symbol('this->params.L_q')   # L_q
lambda_m = Symbol('this->params.lambda_m') # Rotor flux linkage
N_P = Symbol('this->params.N_P') # Number of magnetic pole pairs
J   = Symbol('this->params.J')   # Rotor inertia
alpha_load_pnoise = Symbol('this->params.alpha_load_pnoise') # Load torque process noise
omega_pnoise = Symbol('this->params.omega_pnoise')

# Inputs
u_ab = Matrix(symbols('u_alpha u_beta')) # Stator voltages
u_noise = Symbol('this->params.u_noise') # Additive noise on stator voltage

# Measurements
i_ab_m = Matrix(symbols('i_alpha_m i_beta_m')) # Stator currents observed
i_noise = Symbol('this->params.i_noise') # Additive noise on stator currents

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

u_d_sym, u_q_sym = symbols('u_d, u_q')

T_output = Rational(3,2) * N_P * (lambda_m + (L_d-L_q)*i_d_est) * i_q_est
omega_dot = (T_output - T_l_est)/J

i_d_dot = (L_q*omega_e_est*i_q_est - R_s*i_d_est + u_d_sym)/L_d
i_q_dot = (-L_d*omega_e_est*i_d_est - R_s*i_q_est - lambda_m*omega_e_est + u_q_sym)/L_q

x_dot = Matrix([
    omega_dot,
    omega_e_est,
    i_d_dot,
    i_q_dot,
    0
    ])

subs = {u_d_sym: u_d, u_q_sym:u_q}
#x_dot = x_dot.xreplace(subs)

f = x+dt*x_dot.xreplace(subs)

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

pnoise_sigma = Matrix([omega_pnoise, 0, 0, 0, alpha_load_pnoise*J])*dt

Q += diag(*pnoise_sigma.multiply_elementwise(pnoise_sigma))

# x_p: state vector at time k+1
x_p = rk3(x, x_dot, dt)

# P_p: covariance matrix at time k+1
P_p = F*P*F.T + Q
assert P_p.shape == P.shape
P_p = upperTriangularToVec(P_p)

x_p = x_p.xreplace(subs)
P_p = P_p.xreplace(subs)

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

# Generate code
x_p,P_p,pred_subx = extractSubexpressions([x_p,P_p],'subx',threshold=5)

x_n,P_n,NIS,fuse_subx = extractSubexpressions([x_n,P_n,NIS],'subx',threshold=5)

init_P = upperTriangularToVec(diag(0., math.pi**2, 0., 0., 0.**2))

with open(sys.argv[1],'w') as f:
    f.write(
    '#pragma once\n'
    '#include <stdint.h>\n'
    '\n'
    'struct ekf_state_s {\n'
    '    float x[5];\n'
    '    float P[15];\n'
    '    float innov[2];\n'
    '    float NIS;\n'
    '};\n'
    '\n'
    'struct ekf_params_s {\n'
    '    float R_s;\n'
    '    float L_d;\n'
    '    float L_q;\n'
    '    float lambda_m;\n'
    '    float N_P;\n'
    '    float J;\n'
    '    float alpha_load_pnoise;\n'
    '    float omega_pnoise;\n'
    '    float u_noise;\n'
    '    float i_noise;\n'
    '};\n'
    '\n'
    'struct ekf_obj_s {\n'
    '    struct ekf_params_s params;\n'
    '    struct ekf_state_s state[2];\n'
    '    uint8_t state_idx;\n'
    '};\n'
    '\n'
    'void ekf_init(struct ekf_obj_s* this, struct ekf_params_s* params, float init_theta);\n'
    'void ekf_predict(struct ekf_obj_s* this, float dt, float u_alpha, float u_beta);\n'
    'void ekf_update(struct ekf_obj_s* this, float i_alpha_m, float i_beta_m);\n'
    'struct ekf_state_s* ekf_get_state(struct ekf_obj_s* this);\n'
    )

with open(sys.argv[2],'w') as f:
    f.write(
    '#include "ekf.h"\n'
    '#include <math.h>\n'
    '#include <string.h>\n'
    '#include "helpers.h"\n'
    '\n'
    'struct ekf_state_s* ekf_get_state(struct ekf_obj_s* this) {\n'
    '    return &this->state[this->state_idx];\n'
    '}\n'
    'void ekf_init(struct ekf_obj_s* this, struct ekf_params_s* params, float init_theta) {\n'
    '    float* state = this->state[this->state_idx].x;\n'
    '    float* cov = this->state[this->state_idx].P;\n'
    '    memset(this, 0, sizeof(*this));\n'
    '    memcpy(&this->params, params, sizeof(this->params));\n'
    )

    for i in range(len(init_P)):
        f.write('    cov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(init_P[i])))

    f.write(
    '    state[1] = init_theta;\n'
    '}\n'
    '\n'
    )

    f.write('static float subx[%u];\n' % (max(len(pred_subx),len(fuse_subx)),))
    f.write(
    'void ekf_predict(struct ekf_obj_s* this, float dt, float u_alpha, float u_beta) {\n'
    '    uint8_t next_state_idx = (this->state_idx+1)%2;\n'
    '    float* state = this->state[this->state_idx].x;\n'
    '    float* cov = this->state[this->state_idx].P;\n'
    '    float* state_n = this->state[next_state_idx].x;\n'
    '    float* cov_n = this->state[next_state_idx].P;\n'
    )

    f.write('    // %u operations\n' % (count_ops(x_p)+count_ops(P_p)+count_ops(pred_subx),))

    for i in range(len(pred_subx)):
        f.write('    %s = %s;\n' % (pred_subx[i][0], CCodePrinter_float().doprint(pred_subx[i][1])))

    for i in range(len(x_p)):
        f.write('    state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_p[i])))

    for i in range(len(P_p)):
        f.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_p[i])))

    f.write(
    '\n'
    '    state_n[1] = wrap_2pi(state_n[1]);\n'
    '    this->state_idx = next_state_idx;\n'
    '}\n'
    )

    f.write(
    '\n'
    'void ekf_update(struct ekf_obj_s* this, float i_alpha_m, float i_beta_m) {\n'
    '    uint8_t next_state_idx = (this->state_idx+1)%2;\n'
    '    float* state = this->state[this->state_idx].x;\n'
    '    float* cov = this->state[this->state_idx].P;\n'
    '    float* state_n = this->state[next_state_idx].x;\n'
    '    float* cov_n = this->state[next_state_idx].P;\n'
    '    float* innov = this->state[next_state_idx].innov;\n'
    '    float* NIS = &this->state[next_state_idx].NIS;\n'
    '\n'
    )

    f.write('    // %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(fuse_subx),))
    for i in range(len(fuse_subx)):
        f.write('    %s = %s;\n' % (fuse_subx[i][0], CCodePrinter_float().doprint(fuse_subx[i][1])))

    for i in range(len(x_n)):
        f.write('    state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        f.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))

    for i in range(len(y)):
        f.write('    innov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(y[i])))

    f.write('    *NIS = %s;\n' % (CCodePrinter_float().doprint(NIS[0]),))

    f.write(
    '\n'
    '    state_n[1] = wrap_2pi(state_n[1]);\n'
    '    this->state_idx = next_state_idx;\n'
    '}\n'
    )





