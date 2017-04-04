import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0,20,1000)
y = []
y2 = []
for t in x:
    if t < 3:
        y.append(0.08)
        y2.append(0)
    elif t < 11.5:
        thr = int((t-1)*2)*.025
        y.append(0.08 if int(t*4)%2==0 else thr)
        y2.append(thr)
    else:
        thr = int((t-9.5)*2)*.025
        y.append(thr if int(t*4)%2==0 else -thr)
        y2.append(thr)

plt.plot(x,y,marker='.')
plt.plot(x,y2,marker='.')
plt.show()
