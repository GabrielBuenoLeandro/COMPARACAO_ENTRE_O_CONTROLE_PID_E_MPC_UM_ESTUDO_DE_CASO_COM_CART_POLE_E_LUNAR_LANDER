from matplotlib import pyplot as plt
import numpy as np
from scipy import signal
import gym

m = 0.5 #Massa do pêndulo
u = 1 #Força aplicada
l = 0.65 #Comprimento do pêndulo
x = 1.9524e-1*165 #Aceleração linear*k
o = -2.9775e-1*165 #Aceleração angular*k
M = (1-m*x-m*l*o)/x
print('Massa do carrrinho: ', M)
I = 0.0018537860132090966
g = 9.81

# ml
print(m*l)
a = m*l

# s^2
print((m**2*l**2-(M+m)*(I+m*l**2)))
b = ((m**2*l**2-(M+m)*(I+m*l**2)))
# cte
print((M+m)*m*g*l)
c = ((M+m)*m*g*l)

env = gym.make("CartPole-v1", render_mode="rgb_array")
observation, info = env.reset()

U = [] # Lista para salvar as entradas
Y = []
T = []
P = []

for _ in range(1000):
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)
    U.append(action)
    Y.append(observation[2])
    P.append(action)
    
    if terminated or truncated:
        observation, info = env.reset()
        T.append(_)

        #print(f"Tempo decorrido: {elapsed_time} segundos")

env.close()

w = T[0]

# Defina a função de transferência H(s)
numerator = [a]
denominator = [b, 0, c]  # Exemplo: Sistema de segunda ordem
H = signal.TransferFunction(numerator, denominator)

# Crie uma entrada de degrau
t = np.linspace(0, 10, 1000)  # Cria um vetor de tempo de 0 a 10 segundos
u = np.ones_like(t)  # Cria uma entrada de degrau (todos os valores são 1)
u[500:] = u[500:]*2 # Cria uma entrada de degrau (todos os valores são 1)


for i in range (0, len(U)):
    if U[i]==0:
        U[i]=-1
    else:
        U[i] = 1
# Aplique a entrada à função de transferência para calcular a saída
t, y, x = signal.lsim(H, U=U, T=t)

# t contém o vetor de tempo, y contém a saída

#plt.plot(t[:w], q[:w])

plt.figure(1)
k = np.arange(T[0])
plt.plot(k, U[:w],'b', label='Entrada da F.T')
plt.plot(k, P[:w],'k', label='Entrada do Cartpole')
plt.ylim(-1.1, 1.45)
plt.legend()
plt.xlabel('Amostras')
plt.ylabel('Esquerda<=F [$N$]=>Direita')
plt.title('Entrada')

plt.figure(2)
plt.plot(k, y[:w]+Y[0],'b', label='F.T')
plt.plot(k, Y[:w],'k', label='Cartpole')
plt.xlabel('Amostras')
plt.ylabel('$\\theta (rad)$')
plt.title('Validação da F.T')
plt.grid(False)
plt.legend()
plt.show()