import numpy as np
from math import cos,pi
r=np.random.default_rng(29)
# Numerical regression of the underlying inequality for randomly generated
# *scalar* absolutely continuous theta profiles and their total variation.
for n in range(2000):
    t=np.linspace(0,1,101)
    q=r.normal(size=5)
    theta=q[0]+q[1]*t+q[2]*t**2+q[3]*np.sin(2*pi*t)+q[4]*np.sin(4*pi*t)
    derivative=q[1]+2*q[2]*t+2*pi*q[3]*np.cos(2*pi*t)+4*pi*q[4]*np.cos(4*pi*t)
    # Certified Iw can be chosen as int |d theta| (plus a generous quadrature guard).
    Iw=np.trapz(np.abs(derivative),t)+0.1
    theta2=(abs(theta[0])+abs(theta[-1])+Iw)/2
    assert np.max(abs(theta)) <= theta2+5e-3
roll=1.3549194795981234/cos((1.3549194795981234+0.01104655166042024+0.0021831171404281553)/2)
assert roll<6.281482063079217
print('R29 2-ENDPOINT THEOREM numerical tests PASS (2000 cases); supplied R28 bound',roll)
