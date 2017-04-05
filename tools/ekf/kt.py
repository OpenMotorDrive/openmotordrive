from sympy import *

# scopion
#Vnl = 1.52
#freq = 91.48

# enroute
Vnl = 1.8963
freq = 89.32

omega = freq * 2 * pi

lambda_m = sqrt(Rational(2,3)) * Vnl / omega

Kt = Rational(21,2)*lambda_m

print "lambda_m %f" % (lambda_m.evalf(),)
print "Kt %f" % (Kt.evalf(),)
