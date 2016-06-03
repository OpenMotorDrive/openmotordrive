from sympy import *
from sympy.printing.ccode import *

class MyPrinter(CCodePrinter):
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
            return '*'.join([self._print(expr.base) for _ in range(expr.exp)])
        else:
            return 'powf(%s, %s)' % (self._print(expr.base),
                                 self._print(expr.exp))

def double2float(string):
    import re
    string = re.sub(r"[0-9]+\.[0-9]+", '\g<0>f', string)

    return string



d,q,o,a,b,c,alpha,beta,gamma,theta = symbols('d q o a b c alpha beta gamma elec_theta_m')

T_abc_aby = sqrt(2./3.)*Matrix([[      S.One,      -S.Half,      -S.Half],
                                [     S.Zero,  sqrt(3.)/2., -sqrt(3.)/2.],
                                [ 1./sqrt(2.), 1./sqrt(2.),  1./sqrt(2.)]])

T_aby_dqo = Matrix([[ cos(theta), sin(theta), S.Zero],
                    [-sin(theta), cos(theta), S.Zero],
                    [     S.Zero,     S.Zero,  S.One]])

T_abc_dqo = T_aby_dqo*T_abc_aby

abc_sym = Matrix([[a],[b],[c]])
aby_sym = Matrix([[alpha],[beta],[gamma]])
dqo_sym = Matrix([[d],[q],[o]])

aby = simplify(T_abc_aby*abc_sym)
print "abc->aby"
print double2float(MyPrinter().doprint(aby[0,0], 'alpha'))
print double2float(MyPrinter().doprint(aby[1,0], 'beta'))
print double2float(MyPrinter().doprint(aby[2,0], 'gamma'))

dqo = simplify(T_aby_dqo*aby_sym)
print "aby->dqo"
print double2float(MyPrinter().doprint(dqo[0,0], 'd'))
print double2float(MyPrinter().doprint(dqo[1,0], 'q'))
print double2float(MyPrinter().doprint(dqo[2,0], 'o'))
