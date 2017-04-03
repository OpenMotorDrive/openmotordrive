#!/usr/bin/python
import json
import sys
import numpy as np
import matplotlib.pyplot as plt

with open(sys.argv[1], 'rb') as jsonfile:
    data = json.load(jsonfile)['data']

    t = np.array([(x['t_us']-data[0]['t_us'])*1e-6 for x in data])
    dt = np.array([x['dt'] for x in data])
    encoder_theta_e = np.array([x['encoder_theta_e'] for x in data])
    encoder_omega_e = np.array([x['encoder_omega_e'] for x in data])
    i_alpha_m = np.array([x['i_alpha_m'] for x in data])
    i_beta_m = np.array([x['i_beta_m'] for x in data])
    i_d_m = np.array([x['i_d_m'] for x in data])
    i_q_m = np.array([x['i_q_m'] for x in data])
    u_alpha = np.array([x['u_alpha'] for x in data])
    u_beta = np.array([x['u_beta'] for x in data])
    state = np.array([x['x'] for x in data])
    cov = np.array([x['P'] for x in data])
    theta_e_est = state[:,1]
    i_d_est = state[:,2]
    i_q_est = state[:,3]
    T_l_est = state[:,4]
    theta_e_err = np.array([x['theta_e_err'] for x in data])
    omega_e_est = np.array([x['omega_e_est'] for x in data])
    omega_e_err = np.array([x['omega_e_err'] for x in data])
    NIS = np.array([x['NIS'] for x in data])

    plt.figure(1)
    ax1 = plt.subplot(4,2,1)
    plt.title('electrical rotor angle')
    #plt.fill_between(t, plot_data['theta_e_est_min'], plot_data['theta_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(t, theta_e_est, color='b')
    plt.plot(t, encoder_theta_e, color='g')
    plt.subplot(4,2,2,sharex=ax1)
    plt.title('electrical rotor angular velocity')
    #plt.fill_between(t, plot_data['omega_e_est_min'], plot_data['omega_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(t, omega_e_est, color='b')
    plt.plot(t, encoder_omega_e, color='g')
    plt.subplot(4,2,3,sharex=ax1)
    plt.title('d-axis current')
    #plt.fill_between(t, plot_data['i_d_est_min'], plot_data['i_d_est_max'], facecolor='b', alpha=0.25)
    plt.plot(t, i_d_est, color='b')
    plt.plot(t, i_d_m, color='g')
    plt.subplot(4,2,4,sharex=ax1)
    plt.title('q-axis current')
    #plt.fill_between(t, plot_data['i_q_est_min'], plot_data['i_q_est_max'], facecolor='b', alpha=0.25)
    plt.plot(t, i_q_est, color='b')
    plt.plot(t, i_q_m, color='g')
    plt.subplot(4,2,5,sharex=ax1)
    plt.title('load torque')
    #plt.fill_between(t,plot_data['T_l_est_max'],plot_data['T_l_est_min'],facecolor='b',alpha=.25)
    plt.plot(t, T_l_est, color='b')
    plt.subplot(4,2,6,sharex=ax1)
    plt.title('current fusion normalized innovations squared')
    plt.plot(t, NIS)
    plt.subplot(4,2,7,sharex=ax1)
    plt.title('electrical rotor angle error vs sensor')
    plt.plot(t, theta_e_err)
    plt.subplot(4,2,8,sharex=ax1)
    plt.title('electrical rotor angular velocity error vs sensor')
    plt.plot(t, omega_e_err)
    plt.show()
