from sympy import *
from sympy.printing.ccode import *
from math import sqrt, floor

class CCodePrinter_float(CCodePrinter):
    def __init__(self,settings={}):
        CCodePrinter.__init__(self, settings)
        self.known_functions = {
            "Abs": [(lambda x: not x.is_integer, "fabsf")],
            "gamma": "tgammaf",
            "sin": "sinf",
            "cos": "cosf",
            "tan": "tanf",
            "asin": "asinf",
            "acos": "acosf",
            "atan": "atanf",
            "atan2": "atan2f",
            "exp": "expf",
            "log": "logf",
            "erf": "erff",
            "sinh": "sinhf",
            "cosh": "coshf",
            "tanh": "tanhf",
            "asinh": "asinhf",
            "acosh": "acoshf",
            "atanh": "atanhf",
            "floor": "floorf",
            "ceiling": "ceilf",
        }

    def _print_Pow(self, expr):
        if "Pow" in self.known_functions:
            return self._print_Function(expr)
        PREC = precedence(expr)

        if expr.exp == -1:
            return '1.0/%s' % (self.parenthesize(expr.base, PREC))
        elif expr.exp < 0:
            expr = 1/expr

        if expr.exp == 0.5:
            return 'sqrtf(%s)' % self._print(expr.base)
        elif expr.exp.is_integer and expr.exp <= 4:
            return "(%s)" % ('*'.join(["(%s)" % (self._print(expr.base)) for _ in range(expr.exp)]),)
        else:
            return 'powf(%s, %s)' % (self._print(expr.base),
                                 self._print(expr.exp))

    def _print_Rational(self, expr):
        p, q = int(expr.p), int(expr.q)
        return '%d.0/%d.0' % (p, q)

def toVec(*args):
    ret = Matrix(map(lambda x: Matrix([x]), args)).vec()
    return ret

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

def compressedSymmetricMatrix(prefix, N):
    ret = zeros(N,N)

    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))

    for k in range((N**2-N)/2+N):
        ret[r(k),c(k)] = ret[c(k),r(k)] = Symbol('%s[%u]' % (prefix,k))
    return ret

def count_subexpression(subexpr, expr):
    if hasattr(expr, "__getitem__"):
        return sum(map(lambda x: count_subexpression(subexpr, x), expr))
    else:
        return expr.count(subexpr)

def extractSubexpressions(inexprs, prefix='X', threshold=0, prev_subx=[]):
    subexprs, outexprs = cse(inexprs, symbols=numbered_symbols('__TMP__'), order='none')

    subexprs = prev_subx+subexprs

    for i in reversed(range(len(subexprs))):
        from sympy.logic.boolalg import Boolean
        ops_saved = (count_subexpression(subexprs[i][0], [[x[1] for x in subexprs], outexprs])-1)*subexprs[i][1].count_ops()
        if ops_saved < threshold or isinstance(subexprs[i][1], Boolean):
            sub = dict([subexprs.pop(i)])
            subexprs = map(lambda x: (x[0],x[1].xreplace(sub)), subexprs)
            outexprs = map(lambda x: x.xreplace(sub), outexprs)

    for i in range(len(subexprs)):
        newSym = Symbol('%s[%u]' % (prefix,i+len(prev_subx)))
        sub = {subexprs[i][0]:newSym}
        subexprs[i] = (newSym,subexprs[i][1])
        subexprs = map(lambda x: (x[0],x[1].xreplace(sub)), subexprs)
        outexprs = map(lambda x: x.xreplace(sub), outexprs)

    outexprs = map(lambda x: Matrix(x) if type(x) is ImmutableDenseMatrix else x, outexprs)

    return tuple(outexprs+[subexprs])

# http://www.nt.ntnu.no/users/skoge/prost/proceedings/cdc-ecc-2011/data/papers/1796.pdf

# Parameters
dt = Symbol('dt')
R_m, L_m, K_t, J, N_r = symbols('R_m L_m K_t J N_r')

# Inputs
u_d, u_q, T_l = symbols('u_d u_q T_l')
u_noise, T_l_noise = symbols('u_noise T_l_noise')

# Measurements
i_d_meas, i_q_meas, i_meas_var = symbols('i_d_meas i_q_meas i_meas_var')
z = toVec(i_d_meas, i_q_meas)
R = diag(i_meas_var,i_meas_var)

# States
omega_r, theta_r, i_d, i_q = symbols('omega_r theta_r i_d i_q')
x = toVec(omega_r, theta_r, i_d, i_q)
nStates = len(x)

# Covariance matrix
P = compressedSymmetricMatrix('P', 4)

# Derived variables
omega_e = omega_r*N_r
lambda_0 = 2./3. * K_t / N_r
theta_e = theta_r/N_r

# f: state-transtition model
f = Matrix([
    [omega_r + dt*i_q*K_t/J - T_l/J],
    [i_d + dt*(-R_m/L_m*i_d + omega_e*i_q + 1/L_m*u_d)],
    [dt*(-R_m/L_m*i_q - omega_e*i_d - 1/L_m*lambda_0*omega_e + 1/L_m*u_q)],
    [theta_e + omega_e*dt]])
assert f.shape == x.shape

# F: linearized state-transition model
F = f.jacobian(x)

# u: control input vector
u = toVec(u_d, u_q, T_l)

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

# w_u_sigma: additive noise on u
w_u_sigma = toVec(u_noise, u_noise, T_l_noise)

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# Q: covariance of additive noise on x
Q = G*Q_u*G.T

# P_n: covariance matrix at time k+1
P_n = F*P*F.T + Q
assert P_n.shape == P.shape

#x_n: state vector at time k+1
x_n = f

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

# Optimizations
P_n = upperTriangularToVec(P_n)
x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=2)

for i in range(len(subx)):
    print '%s = %s;' % (subx[i][0], subx[i][1])

for i in range(len(x_n)):
    print 'x_n[%u] = %s;' % (i, CCodePrinter_float().doprint(x_n[i]))

for i in range(len(P_n)):
    print 'P_n[%u] = %s;' % (i, CCodePrinter_float().doprint(P_n[i]))
