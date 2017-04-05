#!/usr/bin/python
from sympy import *
import sys

def _linear_2eq_order1_type1(x, y, t, r, eq):
    l = Dummy('l')
    C1, C2, C3, C4 = ode.get_numbered_constants(eq, num=4)
    l1 = rootof(l**2 - (r['a']+r['d'])*l + r['a']*r['d'] - r['b']*r['c'], l, 0)
    l2 = rootof(l**2 - (r['a']+r['d'])*l + r['a']*r['d'] - r['b']*r['c'], l, 1)
    D = (r['a'] - r['d'])**2 + 4*r['b']*r['c']
    gsol1 = C1*(l1 - r['d'] + r['b'])*exp(l1*t) + C2*(l2 - r['d'] + r['b'])*exp(l2*t)
    gsol2 = C1*(l1 - r['a'] + r['c'])*exp(l1*t) + C2*(l2 - r['a'] + r['c'])*exp(l2*t)
    print D

ode._linear_2eq_order1_type1 = _linear_2eq_order1_type1

R_s, L_q, L_d, omega_e, lambda_r, u_d, u_q, t = symbols('R_s, L_q, L_d, omega_e_est, lambda_r, u_d, u_q, dt')

i_d, i_q = symbols('i_d i_q')
i_dq = Matrix([i_d, i_q])

i_d_0, i_q_0 = symbols('i_d_est i_q_est')
i_dq_0 = Matrix([i_d_0, i_q_0])

i_d_dot = (L_q*omega_e*i_q - R_s*i_d + u_d)/L_d
i_q_dot = (-L_d*omega_e*i_d - R_s*i_q - lambda_r*omega_e + u_q)/L_q

i_dq_dot = Matrix([i_d_dot, i_q_dot])

#dsolve([i_d_eqn, i_q_eqn])

A = Matrix([[        -R_s/L_d, L_q*omega_e/L_d],
            [-L_d*omega_e/L_q, -R_s/L_q       ]])

g = Matrix([u_d/L_d, -lambda_r*omega_e/L_q + u_q/L_q])

eigenval1 = A.eigenvects()[0][0]
eigenvec1 = A.eigenvects()[0][2][0]

eigenval2 = A.eigenvects()[1][0]
eigenvec2 = A.eigenvects()[1][2][0]

a,b = symbols('a b')
soln = (a*exp(eigenval1*t)*eigenvec1 + b*exp(eigenval2*t)*eigenvec2).subs(solve(a*exp(eigenval1*0)*eigenvec1 + b*exp(eigenval2*0)*eigenvec2 - A.inv()*g - i_dq_0, [a,b]))-A.inv()*g

soln = simplify(soln)
#soln = soln.applyfunc(lambda expr: expr.as_real_imag()[0])
#soln = simplify(soln)

print(srepr(soln))
exit()

subs = {
    R_s: .102,
    L_q: 30e-6,
    L_d: 20e-6,
    omega_e: 2e3,
    lambda_r: .0027,
    u_d: 10,
    u_q: 10,
    i_d_0: 5,
    i_q_0: 5,
    }



soln_func = lambdify(t, soln.subs(subs).evalf(), 'numpy')

i_dq_dot_func = lambdify(i_dq, i_dq_dot.subs(subs))

def func(y, t0):
    return flatten(i_dq_dot_func(y[0], y[1]).tolist())

import numpy as np
from scipy.integrate import odeint

times = np.linspace(0, 0.01, 1000)
numerical_ans = odeint(func, list(i_dq_0.subs(subs)), times)
analytical_ans = np.array([flatten(soln_func(t).tolist()) for t in times])

for i in range(1000):
    print numerical_ans[i], analytical_ans[i]


#det = A.det()
#print det
