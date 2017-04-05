#!/usr/bin/python
from subprocess import check_output,call
from scipy.optimize import minimize, basinhopping
import sys
import json
import math
import re

def objective_func(x):
    global objective
    params = dict(init_params)
    for i in range(len(x)):
        params[opt_param_names[i]] = x[i]

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
        obj = math.sqrt(output_data["int_NIS"])*math.sqrt(output_data["var_int"])

    print "obj: %f" % (obj,)

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
    "encoder_delay": (-1e-4,1e-4),
    "i0": (0,10000),
    "a": (0, 10000),
    "b": (0, 10000),
    }

opt_param_names = ["J", "L_d", "lambda_r", "L_q", "R_s", "T_l_pnoise", "i_delay"]
objective="CURRENT_CONSISTENCY"
#opt_param_names = ["i0", "a", "b"]

opt_params = [init_params[x] for x in opt_param_names]
opt_param_bounds = [param_bounds[x] for x in opt_param_names]

res = minimize(objective_func, opt_params, bounds=opt_param_bounds, method='Nelder-Mead', options={'maxiter':1000000,'maxfev':1000000})

print res
objective_func(res.x)

