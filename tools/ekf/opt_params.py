#!/usr/bin/python
from subprocess import check_output,call
from scipy.optimize import minimize, basinhopping
import sys
import json
import math
import re

best_x = None
best_obj = None

def objective_func(x):
    global objective, best_x, best_obj
    params = dict(init_params)
    for i in range(len(x)):
        params[opt_param_names[i]] = x[i]

    #params['L_d'] = params['L_q']*.682

    print params

    param_file_name = "/tmp/param"
    with open(param_file_name, "w+") as f:
        for key, val in params.iteritems():
            f.write("%s %9g\n" % (key, val))

    call(["./a.out", param_file_name, sys.argv[2], "/tmp/output.json"])

    with open("/tmp/output.json", "r") as f:
        output_data = json.load(f)

    for k,v in param_bounds.iteritems():
        if params[k] < v[0] or params[k] > v[1]:
            print k, params[k]
            return sys.float_info.max

    if objective=="RMS_THETA":
        obj = math.sqrt(output_data["theta_ISE"])
    elif objective=="CURRENT_CONSISTENCY":
        obj = math.sqrt(output_data["int_NIS"])
    elif objective=="EVERYTHING":
        obj = math.sqrt(output_data["int_NIS"])*math.sqrt(output_data["theta_ISE"])
    elif objective=="curr_err_sq_int":
        obj = math.sqrt(output_data["curr_err_sq_int"])

    print "obj: %f" % (obj,)

    if best_obj is None or obj < best_obj:
        best_obj = obj
        best_x = x

    return obj

with open('replay_src/ekf.h', 'wb') as f:
    f.write(check_output(["python", "ekf_generator.py"]))

call(["gcc", "-DNO_BULK_DATA", "-fno-trapping-math", "-fno-signaling-nans", "replay_src/main.c", "-lm"])

# load initial parameters from file
init_params = {}
pattern = re.compile("^([A-Za-z0-9_]+)\s+([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)$")
with open(sys.argv[1], 'r') as f:
    for line in f:
        res = pattern.match(line)
        init_params[res.group(1)] = float(res.group(2))


param_bounds = {
    "R_s": (0.01, 1),
    "L_d": (1e-6, 1e-3),
    "L_q": (1e-6, 1e-3),
    "lambda_r": (0,10000),
    "J": (0.000001, 0.01),
    "i_noise": (0.0001, 0.1),
    "u_noise": (0, 20),
    "T_l_pnoise": (0, 100),
    "encoder_theta_e_bias": (-math.pi, math.pi),
    "encoder_delay": (-1e-3,1e-3),
    "omega_pnoise":(0,1e6),
    "param1":(0,1e6),
    "param2":(-1e-4,1e-4),
    "u_d":(0,2),
    "u_ce":(0,2),
    "t_dead_ratio":(0,1),
    }

opt_param_names = ["R_s", "encoder_theta_e_bias", "encoder_delay", "t_dead_ratio"]
objective="curr_err_sq_int"
opt_params = [init_params[x] for x in opt_param_names]
opt_param_bounds = [param_bounds[x] for x in opt_param_names]
res = minimize(objective_func, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})
init_params.update(dict(zip(opt_param_names, res.x)))
#res = minimize(objective_func, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})

#objective="RMS_THETA"
#opt_param_names = ["T_l_pnoise", "u_noise", "encoder_theta_e_bias", "encoder_delay"]
#opt_params = [init_params[x] for x in opt_param_names]
#opt_param_bounds = [param_bounds[x] for x in opt_param_names]
#res = minimize(objective_func, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})
#init_params.update(dict(zip(opt_param_names, res.x)))
#res = minimize(objective_func, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})


print res
objective_func(res.x)
print best_x
