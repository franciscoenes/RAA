import numpy as np
import matplotlib.pyplot as plt

def GerarTrajectoria(p0,pf,v0,vf,tf,dT):
    # Coeficientes do Polinómio
    a0 = p0
    a1 = v0
    a2 = 3/tf**2*(pf-p0)-(2/tf)*v0-(1/tf)*vf 
    a3 = -2/tf**3*(pf-p0)+(1/tf**2)*(vf+v0) 

    # Gerar Vector Temporal
    t = np.arange(0, tf+dT, dT)

    pos = a0 + (a1*t) + (a2*(t**2)) + (a3*(t**3))
    vol = a1 + (2*a2*t) + (3*a3*(t**2))
    acc = (2*a2) + (6*a3*t)

    return t, pos, vol, acc

t,p,v,a = GerarTrajectoria(1,2,3,4,5,0.1)
plt.figure()
plt.plot(t, p)
plt.show()


