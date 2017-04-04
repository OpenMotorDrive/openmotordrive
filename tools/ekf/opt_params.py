from subprocess import check_output,call
from scipy.optimize import minimize, basinhopping
import sys
import json
import math

with open('replay_src/ekf.h', 'wb') as f:
    f.write(check_output(["python", "ekf_generator.py"]))

call(["gcc", "-DNO_BULK_DATA", "-fno-trapping-math", "-fno-signaling-nans", "replay_src/main.c", "-lm"])

init_params = {
    "R_s": .102,
    "L_d": 28e-6,
    "L_q": 44e-6,
    "lambda_r": 0.00215917953167269,
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

init_params = {'u_noise': 2.0, 'L_q': 3.9189823237151104e-05, 'R_s': 0.062943814165774464, 'T_l_pnoise': 0.0020896777748964385, 'J': 2.3263752969425543e-05, 'N_P': 7, 'i_noise': 0.0027047604664784518, 'i_delay': 5.0480600093122016e-07, 'lambda_r': 0.00215917953167269, 'encoder_delay': 1.8007851480745154e-05, 'L_d': 3.1947579451976768e-05, 'theta_pnoise': 1.0718124170948967e-08, 'omega_pnoise': 5.9985332333787525e-07, 'encoder_theta_e_bias': -0.2594093644521529}

param_bounds = {
    "R_s": (0.01, 1),
    "L_d": (20e-6, 1e-4),
    "L_q": (20e-6, 1e-4),
    "lambda_r": (0,10000),
    "J": (0.000001, 0.001),
    "i_noise": (0.001, 0.05),
    "u_noise": (0, 20),
    "T_l_pnoise": (0, 100),
    "omega_pnoise": (0, 1e6),
    "theta_pnoise": (0, 1e6),
    "encoder_theta_e_bias": (-math.pi, math.pi),
    "i_delay": (-1e-4,1e-4),
    "encoder_delay": (-1e-4,1e-4)
    }

opt_param_names = ["J", "L_d", "L_q", "lambda_r", "R_s", "i_noise", "T_l_pnoise", "omega_pnoise", "theta_pnoise", "i_delay"]
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

    return math.sqrt(output_data["int_NIS"])*math.sqrt(output_data["var_int"])#*params['u_noise']*math.sqrt(output_data["int_NIS"])#*max(math.sqrt(output_data["int_NIS"]),1.)

res = minimize(f, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})

print res
f(res.x)

