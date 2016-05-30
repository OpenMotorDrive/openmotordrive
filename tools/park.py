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



d,q,o,a,b,c,theta = symbols('d q o a b c theta')

K = sqrt(2./3.)*Matrix([[ cos(theta),  cos(theta - 2.*S.Pi/3.),  cos(theta + 2.*S.Pi/3.)],
                           [-sin(theta), -sin(theta - 2.*S.Pi/3.), -sin(theta + 2.*S.Pi/3.)],
                           [  sqrt(2)/2,                sqrt(2)/2,                sqrt(2)/2]])

abc = Matrix([[a],[b],[c]])
dqo = Matrix([[d],[q],[o]])

dqo_soln = K*abc
print "dqo"
print double2float(MyPrinter().doprint(simplify(dqo_soln[0,0]), 'd'))
print double2float(MyPrinter().doprint(simplify(dqo_soln[1,0]), 'q'))
print double2float(MyPrinter().doprint(simplify(dqo_soln[2,0]), 'o'))

abc_soln = K.T*dqo
#abc_soln = abc_soln.subs(o,S.Zero)
print "abc"
print double2float(MyPrinter().doprint(abc_soln[0,0], 'a'))
print double2float(MyPrinter().doprint(abc_soln[1,0], 'b'))
print double2float(MyPrinter().doprint(abc_soln[2,0], 'c'))
