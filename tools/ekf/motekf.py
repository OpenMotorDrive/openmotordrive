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
lambda_r = 2./3. * K_t/N_P # Rotor flux linkage - Ohm, section III, eqn 3.7

# Inputs
u_ab = Matrix(symbols('u_alpha u_beta')) # Stator voltages
u_noise = Symbol('u_noise') # Additive noise on stator voltage

# Measurements
i_ab_m = Matrix(symbols('i_alpha_m i_beta_m')) # Stator currents observed
i_noise = Symbol('i_noise') # Additive noise on stator currents
#theta_e_m = Symbol('theta_e_m')
#theta_e_m_noise = Symbol('theta_e_m_noise')
z = toVec(i_ab_m) # Observation vector
R = diag(i_noise**2,i_noise**2) # Covariance of observation vector

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
Q += diag(0**2, 0**2, 0**2, 0**2, T_l_pnoise**2)

x_p = f

# P_p: covariance matrix at time k+1
P_p = F*P*F.T + Q
assert P_p.shape == P.shape

# h: predicted measurement
h = zeros(2,1)
h[0:2,0] = R_dq_ab(theta_e_est) * Matrix([i_d_est, i_q_est])
#h[2,0] = theta_e_est

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

P_p = upperTriangularToVec(P_p)
P_n = upperTriangularToVec(P_n)

def print_code():
    global x_n, P_n

    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=5)

    print count_ops(x_n)+count_ops(P_n)+count_ops(subx)

    init_P = upperTriangularToVec(diag(100., math.pi**2, i_noise**2, i_noise**2, 0.1**2))

    soln = solve([i_d_dot-Symbol('i_d_dot'), i_q_dot-Symbol('i_q_dot')], [u_d,u_q])
    print(CCodePrinter_float().doprint(soln[u_d]))
    print(CCodePrinter_float().doprint(soln[u_q]))

    for i in range(len(init_P)):
        print('cov[%u] = %s;' % (i, CCodePrinter_float().doprint(init_P[i])))

    print 'static float subx[%u];' % (len(subx),)
    for i in range(len(subx)):
        print('%s = %s;' % (subx[i][0], CCodePrinter_float().doprint(subx[i][1])))

    for i in range(len(x_n)):
        print('state_n[%u] = %s;' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        print('cov_n[%u] = %s;' % (i, CCodePrinter_float().doprint(P_n[i])))

def test_ekf():
    global x_n, P_n, dt, NIS, S, y
    from scipy.io import loadmat
    import numpy as np
    data = loadmat('mot_data.mat')

    PREDICTION_ONLY = False
    TRUTH_ANGLE_OVERRIDE = False

    if PREDICTION_ONLY:
        print("PREDICTION_ONLY")
    if TRUTH_ANGLE_OVERRIDE:
        print("TRUTH_ANGLE_OVERRIDE")

    if PREDICTION_ONLY:
        x_n = x_p
        P_n = P_p

    subs = {
        R_s:0.102,
        L_d:28.0*1e-6,
        L_q:43.0*1e-6,
        K_v:360.,
        J:0.00003,
        N_P:7,
        i_noise: 0.0075,
        u_noise: 0.6,
        T_l_pnoise: 60.*dt,
        }

    x_n = x_n.xreplace(subs).xreplace(subs)
    P_n = P_n.xreplace(subs).xreplace(subs)
    NIS = NIS.xreplace(subs).xreplace(subs)
    S = S.xreplace(subs).xreplace(subs)
    y = y.xreplace(subs).xreplace(subs)

    pprint(Q.xreplace(subs).xreplace(subs))

    x_n,P_n,NIS,S,y,subx = extractSubexpressions([x_n,P_n,NIS,S,y],'subx',threshold=1)

    from sympy.utilities.autowrap import ufuncify
    lambda_args = (zip(*subx)[0], x, upperTriangularToVec(P), dt, i_ab_m[0], i_ab_m[1], u_ab[0], u_ab[1])

    subx_lambda = []

    for sym,expr in subx:
        subx_lambda.append(lambdify(lambda_args, expr, 'numpy'))

    x_n_lambda = lambdify(lambda_args, x_n, 'numpy')
    P_n_lambda = lambdify(lambda_args, P_n, 'numpy')
    NIS_lambda = lambdify(lambda_args, NIS, 'numpy')
    S_lambda = lambdify(lambda_args, S, 'numpy')
    y_lambda = lambdify(lambda_args, y, 'numpy')

    init_P = upperTriangularToVec(diag(10.**2, (math.pi)**2, 0.01**2, 0.01**2, 0.1**2))

    curr_x = np.array([0.,data['theta_e'][0][0], 0., 0., 0.])
    curr_P = np.array(init_P.T)
    curr_subx = np.zeros(len(subx_lambda))


    plot_data = {}

    def add_plot_data(name, val):
        if name not in plot_data.keys():
            plot_data[name] = []
        plot_data[name].append(val)

    n_samples = len(data['dt'][0])
    n_process = n_samples
    poison = False
    for i in range(n_process):
        try:
            t = (data['t_us'][0][i]-data['t_us'][0][0])*1e-6
            data['t_us'][0][i]
            sys.stdout.write('\r%fs %u%%' % (t,int(round(100*float(i)/n_process))))
            sys.stdout.flush()
            theta_e_truth = data['theta_e'][0][i]
            omega_e_truth = data['omega_e'][0][i]
            dt, i_alpha_m, i_beta_m, u_alpha, u_beta = (data['dt'][0][i], data['i_alpha'][0][i], data['i_beta'][0][i], data['u_alpha'][0][i], data['u_beta'][0][i])

            lambda_args = (curr_subx, curr_x, curr_P, dt, i_alpha_m, i_beta_m, u_alpha, u_beta)
            for j in range(len(subx)):
                curr_subx[j] = subx_lambda[j](*lambda_args)
            next_x = x_n_lambda(*lambda_args)
            next_P = P_n_lambda(*lambda_args)

            obs_S = S_lambda(*lambda_args)
            obs_y = y_lambda(*lambda_args)
            obs_NIS = NIS_lambda(*lambda_args)[0][0]

            if TRUTH_ANGLE_OVERRIDE:
                next_x[1][0] = theta_e_truth

            while next_x[1][0] > 2*math.pi:
                next_x[1][0] -= 2*math.pi
            while next_x[1][0] < 0:
                next_x[1][0] += 2*math.pi

            next_P_uncompressed = uncompressSymMatrix(next_P)

            theta_e_mu = next_x[1][0]
            theta_e_sigma = float(next_P_uncompressed[1,1]**0.5)
            T_l_mu = next_x[4][0]
            T_l_sigma = float(next_P_uncompressed[4,4]**0.5)
            omega_e_mu = next_x[0][0]*7
            omega_e_sigma = float(next_P_uncompressed[0,0]**0.5 * 7)
            i_d_mu = next_x[2][0]
            i_d_sigma = float(next_P_uncompressed[2,2]**0.5)
            i_q_mu = next_x[3][0]
            i_q_sigma = float(next_P_uncompressed[3,3]**0.5)

            u_dq_truth = R_ab_dq(theta_e_truth) * Matrix([u_alpha, u_beta])
            i_dq_truth = (R_ab_dq(theta_e_truth) * Matrix([i_alpha_m, i_beta_m]))

            add_plot_data('t', t)

            add_plot_data('theta_e_est', theta_e_mu)
            add_plot_data('theta_e_est_min', theta_e_mu-theta_e_sigma)
            add_plot_data('theta_e_est_max', theta_e_mu+theta_e_sigma)
            add_plot_data('theta_e_truth',theta_e_truth)
            add_plot_data('theta_e_truth_min',theta_e_truth-0.1)
            add_plot_data('theta_e_truth_max',theta_e_truth+0.1)

            add_plot_data('omega_e_est',omega_e_mu)
            add_plot_data('omega_e_est_min',omega_e_mu-omega_e_sigma)
            add_plot_data('omega_e_est_max',omega_e_mu+omega_e_sigma)
            add_plot_data('omega_e_truth',omega_e_truth)
            add_plot_data('omega_e_truth_min',omega_e_truth-50.)
            add_plot_data('omega_e_truth_max',omega_e_truth+50.)

            add_plot_data('T_l_est',T_l_mu)
            add_plot_data('T_l_est_min',T_l_mu-T_l_sigma)
            add_plot_data('T_l_est_max',T_l_mu+T_l_sigma)

            add_plot_data('i_d_est', i_d_mu)
            add_plot_data('i_d_est_min', i_d_mu-i_d_sigma)
            add_plot_data('i_d_est_max', i_d_mu+i_d_sigma)
            add_plot_data('i_d_truth', i_dq_truth[0])
            add_plot_data('i_d_truth_min', i_dq_truth[0]-i_noise.xreplace(subs))
            add_plot_data('i_d_truth_max', i_dq_truth[0]+i_noise.xreplace(subs))

            add_plot_data('i_q_est', i_q_mu)
            add_plot_data('i_q_est_min', i_q_mu-i_q_sigma)
            add_plot_data('i_q_est_max', i_q_mu+i_q_sigma)
            add_plot_data('i_q_truth', i_dq_truth[1])
            add_plot_data('i_q_truth_min', i_dq_truth[1]-i_noise.xreplace(subs))
            add_plot_data('i_q_truth_max', i_dq_truth[1]+i_noise.xreplace(subs))

            add_plot_data('omega_e_err', omega_e_mu-omega_e_truth)

            add_plot_data('theta_e_err', wrap_pi(theta_e_mu-theta_e_truth))

            add_plot_data('NIS',obs_NIS)

            curr_x = next_x
            curr_P = next_P
            if poison:
                break
        except KeyboardInterrupt:
            poison=True

    sys.stdout.write('\n')
    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.subplot(4,2,1)
    plt.title('electrical rotor angle')
    plt.fill_between(plot_data['t'], plot_data['theta_e_est_min'], plot_data['theta_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['theta_e_est'], color='b')
    plt.plot(plot_data['t'], plot_data['theta_e_truth'], color='g')
    plt.subplot(4,2,2)
    plt.title('electrical rotor angular velocity')
    plt.fill_between(plot_data['t'], plot_data['omega_e_est_min'], plot_data['omega_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['omega_e_est'], color='b')
    plt.plot(plot_data['t'], plot_data['omega_e_truth'], color='g')
    plt.subplot(4,2,3)
    plt.title('d-axis current')
    plt.fill_between(plot_data['t'], plot_data['i_d_est_min'], plot_data['i_d_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['i_d_est'], color='b')
    plt.plot(plot_data['t'], plot_data['i_d_truth'], color='g')
    plt.subplot(4,2,4)
    plt.title('q-axis current')
    plt.fill_between(plot_data['t'], plot_data['i_q_est_min'], plot_data['i_q_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['i_q_est'], color='b')
    plt.plot(plot_data['t'], plot_data['i_q_truth'], color='g')
    plt.subplot(4,2,5)
    plt.title('load torque')
    plt.fill_between(plot_data['t'],plot_data['T_l_est_max'],plot_data['T_l_est_min'],facecolor='b',alpha=.25)
    plt.plot(plot_data['t'], plot_data['T_l_est'], color='b')
    plt.subplot(4,2,6)
    plt.title('current fusion normalized innovations squared')
    plt.plot(plot_data['t'], plot_data['NIS'])
    plt.subplot(4,2,7)
    plt.title('electrical rotor angle error vs sensor')
    plt.plot(plot_data['t'], plot_data['theta_e_err'])
    plt.subplot(4,2,8)
    plt.title('electrical rotor angular velocity error vs sensor')
    plt.plot(plot_data['t'], plot_data['omega_e_err'])


    plt.show()

print_code()
#test_ekf()
