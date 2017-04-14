# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from sympy import *
from sympy.printing.ccode import *

class CCodePrinter_float(CCodePrinter):
    def __init__(self,settings={}):
        CCodePrinter.__init__(self, settings)
        self.known_functions = {
            "Abs": [(lambda x: not x.is_integer, "fabsf")],
            "gamma": "tgammaf",
            "sin": "sinf_fast",
            "cos": "cosf_fast",
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



d,q,o,a,b,c,alpha,beta,gamma,theta,max_duty,omega = symbols('d q o a b c alpha beta gamma elec_theta_m max_duty omega')

T_abc_aby = Rational(2,3) * Matrix([[            1, -Rational(1,2), -Rational(1,2)],
                                    [            0,      sqrt(3)/2,     -sqrt(3)/2],
                                    [Rational(1,2),  Rational(1,2),  Rational(1,2)]])

T_aby_abc = Matrix([[             1,          0, 1],
                    [-Rational(1,2),  sqrt(3)/2, 1],
                    [-Rational(1,2), -sqrt(3)/2, 1]])

#T_abc_aby = sqrt(2./3.)*Matrix([[      S.One,      -S.Half,      -S.Half],
                                #[     S.Zero,  sqrt(3)/2, -sqrt(3)/2],
                                #[ 1/sqrt(2), 1/sqrt(2),  1/sqrt(2)]])

#T_aby_abc = T_abc_aby.inv()


T_aby_dqo = Matrix([[ cos(theta), sin(theta), S.Zero],
                    [-sin(theta), cos(theta), S.Zero],
                    [     S.Zero,     S.Zero,  S.One]])

T_dqo_aby = T_aby_dqo.inv()

T_abc_dqo = T_aby_dqo*T_abc_aby

abc_sym = Matrix([[a],[b],[c]])
aby_sym = Matrix([[alpha],[beta],[gamma]])
dqo_sym = Matrix([[d],[q],[o]])

aby = simplify(T_abc_aby*abc_sym).evalf()
print "abc->aby"
print CCodePrinter_float().doprint(aby[0,0], 'alpha')
print CCodePrinter_float().doprint(aby[1,0], 'beta')

abc = simplify(T_aby_abc*aby_sym.subs(gamma, 0)).evalf()
print "\naby->abc"
print CCodePrinter_float().doprint(abc[0,0], 'a')
print CCodePrinter_float().doprint(abc[1,0], 'b')
print CCodePrinter_float().doprint(abc[2,0], 'c')

dqo = simplify(T_aby_dqo*aby_sym).evalf()
print "\naby->dqo"
print CCodePrinter_float().doprint(dqo[0,0], 'd')
print CCodePrinter_float().doprint(dqo[1,0], 'q')

aby = simplify(T_dqo_aby*dqo_sym).evalf()
print "\ndqo->aby"
print CCodePrinter_float().doprint(aby[0,0], 'alpha')
print CCodePrinter_float().doprint(aby[1,0], 'beta')

#delta_theta = Symbol('delta_theta')
#pprint(simplify((T_dqo_aby.subs(theta, theta+delta_theta) * T_aby_dqo * aby_sym).subs(theta, 0))[0:2,0])
