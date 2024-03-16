import numpy as np
import matplotlib.pyplot as plt
pr = [1, 1, 1, 0, 0, 1, 1]
rb = np.array([0, 1, 2, 2.00001, 3, 3.00001, 4])*1e-3
pra = [1, 1, 0, 1]
rba = np.array([0, 1, 2.00001, 3.00001])*1e-3
plt.plot(rb, pr, 'b')
plt.plot(rba, pra, 'xr')
plt.title('Sinal PRBS')
plt.ylabel(r'u')
plt.xlabel('Tempo(s)')
plt.show()