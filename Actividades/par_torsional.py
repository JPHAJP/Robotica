import numpy as np
import matplotlib.pyplot as plt

#T_ab
r=(30/2)*10**-3
J=(1/2*np.pi)*r**4
print(J)
T=300
c=r

T_max1=(T*c)/J
#Resultado en MPa
print("El esfuerzo maximo es:")
print(np.round(T_max1/10**6,2), " MPa")

#T_bc
r=(46/2)*10**-3
J=1/2*np.pi*r**4
print(J)
T=400+300
c=r

T_max2=(T*c)/J
print(np.round(T_max2/10**6,2), " MPa")