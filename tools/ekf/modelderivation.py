from sympy import *
from sympy.physics.vector import dynamicsymbols
import sys

def copy_upper_to_lower_offdiagonals(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols

    ret = M[:,:]

    for r in range(ret.rows):
        for c in range(ret.cols):
            if r > c:
                ret[r,c] = ret[c,r]
    return ret

theta_e = dynamicsymbols('theta_e')
theta_e_dot = dynamicsymbols('theta_e',1)
omega_e = dynamicsymbols('omega_e')
theta = theta_e

#T_abc_dqo_ohm = sqrt(Rational(2,3))*Matrix([[cos(theta_e), cos(theta_e-2*pi/3), cos(theta_e+2*pi/3)],
                                            #[sin(theta_e), sin(theta_e-2*pi/3), sin(theta_e+2*pi/3)],
                                            #[1/sqrt(2), 1/sqrt(2), 1/sqrt(2)]])

#T_dqo_abc_ohm = sqrt(Rational(2,3))*Matrix([[cos(theta_e), cos(theta_e-2*pi/3), cos(theta_e+2*pi/3)],
                                            #[sin(theta_e), sin(theta_e-2*pi/3), sin(theta_e+2*pi/3)],
                                            #[1/sqrt(2), 1/sqrt(2), 1/sqrt(2)]]).T

T_abc_dqo = Rational(2,3)*Matrix([[cos(theta_e), cos(theta_e-2*pi/3), cos(theta_e+2*pi/3)],
                                  [-sin(theta_e), -sin(theta_e-2*pi/3), -sin(theta_e+2*pi/3)],
                                  [1/sqrt(2), 1/sqrt(2), 1/sqrt(2)]])

T_dqo_abc = Matrix([[cos(theta_e), cos(theta_e-2*pi/3), cos(theta_e+2*pi/3)],
                    [-sin(theta_e), -sin(theta_e-2*pi/3), -sin(theta_e+2*pi/3)],
                    [1/sqrt(2), 1/sqrt(2), 1/sqrt(2)]]).T

I_abc = Matrix(dynamicsymbols('I_(a:c)'))
lambda_m = Symbol('lambda_m')
omega_e = Symbol('omega_e')
L_so, L_sl, L_x = symbols('L_so L_sl L_x')
R_s = Symbol('R_s')
N_P = Symbol('N_P')
t = Symbol('t')
lambda_dqo_sym = Matrix(dynamicsymbols('lambda_d lambda_q lambda_o'))
V_dqo_sym = Matrix(symbols('V_d V_q V_o'))
I_dqo_sym = Matrix(dynamicsymbols('I_d I_q I_o'))
I_dqo_dot_sym = Matrix(dynamicsymbols('I_d I_q I_o', 1))
L_dqo_sym = Matrix(symbols('L_d L_q L_o'))


# 2.13 .. 2.15
lambda_m_abc = Matrix([
    lambda_m*cos(theta_e),
    lambda_m*cos(theta_e-2*pi/3),
    lambda_m*cos(theta_e+2*pi/3)
    ])

# 2.7 .. 2.12
L_matrix = zeros(3,3)

L_matrix[0,0] = L_so + L_sl + L_x * cos(2*theta_e)
L_matrix[1,1] = L_so + L_sl + L_x * cos(2*theta_e + 2*pi/3)
L_matrix[2,2] = L_so + L_sl + L_x * cos(2*theta_e - 2*pi/3)
L_matrix[0,1] = -L_so/2 + L_x * cos(2*theta_e - 2*pi/3)
L_matrix[1,2] = -L_so/2 + L_x * cos(2*theta_e)
L_matrix[0,2] = -L_so/2 + L_x * cos(2*theta_e + 2*pi/3)

L_matrix = copy_upper_to_lower_offdiagonals(L_matrix)

#L_matrix = L_matrix * T_dqo_abc
#pprint(simplify(L_matrix))
#sys.exit()

# 2.4 .. 2.6
lambda_abc = L_matrix * I_abc + lambda_m_abc
lambda_abc = lambda_abc.subs(dict(zip(I_abc, T_dqo_abc*I_dqo_sym)))

V_abc = R_s * I_abc + diff(lambda_abc,t)
V_abc = V_abc.subs(dict(zip(I_abc, T_dqo_abc*I_dqo_sym)))

# put V_dqo in terms of L_d and L_q
V_dqo = simplify(T_abc_dqo * V_abc).as_mutable()
V_dqo[0] = collect(collect(V_dqo[0], I_dqo_dot_sym[0]), I_dqo_sym[1]*diff(theta_e,t))
V_dqo[1] = collect(collect(V_dqo[1], I_dqo_dot_sym[1]), I_dqo_sym[0]*diff(theta_e,t))
V_dqo = V_dqo.subs([(L_sl+3*L_so/2+3*L_x/2, L_dqo_sym[0]), (L_sl+3*L_so/2-3*L_x/2, L_dqo_sym[1]), (theta_e_dot, omega_e)])

# solve for I_dqo_dot
I_dqo_dot_soln = solve(V_dqo-V_dqo_sym, I_dqo_dot_sym)
I_dqo_dot = Matrix([I_dqo_dot_soln[k] for k in I_dqo_dot_sym])

# solve for torque
lambda_dqo = simplify(T_abc_dqo * lambda_abc)
P_o = Rational(3,2) * (-omega_e*lambda_dqo[1]*I_dqo_sym[0] + omega_e*lambda_dqo[0]*I_dqo_sym[1])
T = N_P * P_o/omega_e
T = simplify(T.subs([(L_x, (L_dqo_sym[1]-L_dqo_sym[0])/2), (theta_e_dot, omega_e)]))
print "\n################## V_dqo ##################"
pprint(V_dqo)

print "\n################ I_dqo_dot ################"
pprint(I_dqo_dot)

print "\n################# Power #################"
pprint(P_o)

print "\n################# Torque #################"
pprint(T)
