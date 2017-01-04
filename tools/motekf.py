from sympy import *
from math import sqrt, floor

def upperTriangularToVec(M):
    assert M.rows == M.cols

    N = M.rows
    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))
    return Matrix([M[r(k),c(k)] for k in range((N**2-N)/2+N)])

def copy_upper_to_lower_offdiagonals(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols

    ret = M[:,:]

    for r in range(ret.rows):
        for c in range(ret.cols):
            if r > c:
                ret[r,c] = ret[c,r]
    return ret

# http://www.nt.ntnu.no/users/skoge/prost/proceedings/cdc-ecc-2011/data/papers/1796.pdf

dt = Symbol('dt')
R_m, L_m, K_m, J, N_r = symbols('R_m L_m K_m J N_r')
lambda_0 = 2./3. * K_m / N_r

# z: measurement vector
z = Matrix(symbols('z[0:2]'))

# R: measuremnt covariance
R = Matrix(2,2,symbols('R[0:2][0:2]'))
R = copy_upper_to_lower_offdiagonals(R)

# x: state vector
x = Matrix(symbols('x[0:4]'))

# P: state covariance
P = Matrix(4,4,symbols('P[0:4][0:4]'))
P = copy_upper_to_lower_offdiagonals(P)

# u: control input vector
u = Matrix(symbols('u[0:3]'))

# w_u_sigma: additive noise on u
w_u_sigma = Matrix(symbols('w_u_sigma[0:3]'))

omega_r = x[0]
omega_e = omega_r*N_r
i_d = x[1]
i_q = x[2]
theta_r = x[3]
u_d = u[0]
u_q = u[1]
T_l = u[2]

i_d_m = z[0]
i_q_m = z[1]

# f: state transition model
f = Matrix([
    [omega_r + dt*i_q*K_m/J - T_l/J],
    [i_d + dt*(-R_m/L_m*i_d + omega_e*i_q + 1/L_m*u_d)],
    [dt*(-R_m/L_m*i_q - omega_e*i_d - 1/L_m*lambda_0*omega_e + 1/L_m*u_q)],
    [theta_r + omega_r*dt]])

# F: linearized state transition model
F = f.jacobian(x)

# G: control-influence matrix
G = f.jacobian(u)

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# Q: covariance of additive noise on x
Q = G*Q_u*G.T

# x_n: predicted state
x_n = simplify(f)

# P_n: predicted covariance
P_n = simplify(F*P*F.T + Q)

# h: predicted measurement
h = Matrix([i_d, i_q])

# y: innovation vector
y = z-h

# H: measurement sensitivity matrix
H = h.jacobian(x).xreplace(dict(zip(x,x_n)))

# S: innovation covariance
S = H*P*H.T + R

# K: Kalman gain
K = P*H.T*S.inv()

x_n = x_n + K*y
P_n = (eye(4)-K*H)*P_n
P_n = upperTriangularToVec(P_n)

#pprint(x_n)
#print count_ops(x_n)
#pprint(P_n)
#print count_ops(P_n)

subx, outx = cse([x_n, P_n], numbered_symbols('subx'))

print len(subx)
