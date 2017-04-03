from subprocess import call
from scipy.optimize import minimize, basinhopping
import sys
import json
import math


print call(["gcc", "-DNO_BULK_DATA", "-fno-trapping-math", "-fno-signaling-nans", "replay_src/main.c", "-lm"])

init_params = {
    "R_s": .102,
    "L_d": 28e-6,
    "L_q": 44e-6,
    "K_v": 360,
    "J": 0.000031,
    "N_P": 7,
    "i_noise": 0.01,
    "u_noise": .9,
    "T_l_pnoise": 0.01,
    "omega_pnoise": 0,
    "theta_pnoise": 0,
    "encoder_theta_e_bias": -.23,
    "i_delay": 0,
    "encoder_delay": 0,
    }

init_params = {'u_noise': 4.0004704107202, 'L_q': 4.4206497706787936e-05, 'R_s': 0.093210521872504837, 'T_l_pnoise': 0.01, 'J': 3.1449278655591416e-05, 'N_P': 7, 'i_noise': 0.0016308402307166165, 'i_delay': 4.9159942354516481e-07, 'K_v': 400, 'encoder_delay': 1.8007851480745154e-05, 'L_d': 2.1483962910302035e-05, 'theta_pnoise': 6.7663738586479169e-09, 'omega_pnoise': 0.00021543049985731788, 'encoder_theta_e_bias': -0.2594093644521529}

param_bounds = {
    "R_s": (0.01, 1),
    "L_d": (20e-6, 1e-4),
    "L_q": (20e-6, 1e-4),
    "K_v": (0,10000),
    "J": (0.000001, 0.001),
    "i_noise": (0.001, 0.05),
    "u_noise": (0, 5),
    "T_l_pnoise": (0, 100),
    "omega_pnoise": (0, 1e6),
    "theta_pnoise": (0, 1e6),
    "encoder_theta_e_bias": (-math.pi, math.pi),
    "i_delay": (-1e-4,1e-4),
    "encoder_delay": (-1e-4,1e-4)
    }

opt_param_names = ["u_noise", "J", "L_d", "L_q", "K_v", "R_s", "i_noise", "T_l_pnoise", "omega_pnoise", "theta_pnoise", "i_delay"]
opt_params = [init_params[x] for x in opt_param_names]
opt_param_bounds = [param_bounds[x] for x in opt_param_names]

def f(x):
    params = dict(init_params)
    for i in range(len(x)):
        params[opt_param_names[i]] = x[i]

    print params

    param_file_name = "/tmp/param"
    with open(param_file_name, "w+") as f:
        for key, val in params.iteritems():
            f.write("%s %9g\n" % (key, val))

    call(["./a.out", param_file_name, sys.argv[1], "/tmp/output.json"])

    with open("/tmp/output.json", "r") as f:
        output_data = json.load(f)

    for k,v in param_bounds.iteritems():
        if params[k] < v[0] or params[k] > v[1]:
            print k, params[k]
            return sys.float_info.max

    return output_data["int_NIS"]*math.sqrt(output_data["var_int"])#*params['u_noise']*math.sqrt(output_data["int_NIS"])#*max(math.sqrt(output_data["int_NIS"]),1.)

res = minimize(f, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})

print res
f(res.x)

