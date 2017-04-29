from math import *
import matplotlib.pyplot as plt
import numpy as np
from sympy import *

import sys

def svgen(alpha, beta, max_duty=1.):
    Va = alpha
    Vb = (-(alpha/2.0)+(beta*sqrt(3.0)/2.0))
    Vc = (-(alpha/2.0)-(beta*sqrt(3.0)/2.0))

    Vneutral = 0.5 * (max(Va,Vb,Vc) + min(Va,Vb,Vc))

    Va += 0.5*max_duty-Vneutral
    Vb += 0.5*max_duty-Vneutral
    Vc += 0.5*max_duty-Vneutral

    return (min(max(Va,0.),max_duty), min(max(Vb,0.),max_duty), min(max(Vc,0.),max_duty), Vneutral)

print svgen(1, 0.)

sys.exit()

times = np.linspace(0,6.28,1000)
voltages = zip(*[svgen(sin(t) * sqrt(2./3.), sin(t+pi/2) * sqrt(2./3.),1.) for t in times])

print times, voltages

plt.plot(times, np.array(voltages[1])-np.array(voltages[0]))
plt.plot(times, np.array(voltages[2])-np.array(voltages[1]))
plt.plot(times, np.array(voltages[0])-np.array(voltages[2]))

#plt.plot(times, voltages[0])
#plt.plot(times, voltages[1])
#plt.plot(times, voltages[2])
#plt.plot(times, voltages[3])
plt.show()
