import matplotlib.pyplot as plt
import numpy as np
import control


# Definindo o sistema
num = [0.325]  # numerador
den = [-0.006612887163570405, 0, 1.679193272178997]  # denominador
sys = control.TransferFunction(num, den)

# Resposta ao degrau
T, yout = control.step_response(sys)

# Plotando a resposta ao degrau
plt.figure()
plt.plot(np.arange(len(yout)), yout,'g')
plt.title('Resposta ao Degrau')
plt.xlabel('Amostras')
plt.ylabel('$\\theta$')
plt.grid(False)