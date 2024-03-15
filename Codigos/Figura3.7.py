import gym
from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import linregress


# Gráfico: "Resultados da aplicação de uma força constante à direita." 

env = gym.make("CartPole-v1", render_mode="human")
env.action_space.seed(42)
observation, info = env.reset(seed=42)
t = [] # Lista o episódio
P = [] # Lista para a Posição
V = [] # Lista para a Velocidade
A = [] # Lista para o ângulo
Va = [] # Lista para a Velocidade angular
for _ in range(100):
    observation, reward, terminated, truncated, info = env.step(1) # F esquerda
    P.append(observation[0]) # Guarda a Posição
    V.append(observation[1]) # Guarda a Velocidade
    A.append(observation[2]) # Guarda  o ângulo
    Va.append(observation[3]) # Guarda a Velocidade angular

    if terminated or truncated:
        observation, info = env.reset()
        t.append(_) # Guarda o término de cada episódio

env.close()

# Dados de exemplo
x = np.arange(0, t[0])
Vl = np.array(V[:t[0]])
Vt = np.array(Va[:t[0]])

# Aplicando a regressão linear
resultl = linregress(x, Vl)
resultt = linregress(x, Vt)

I = np.zeros(10000) #=> Vetor que recebe os valores de inércia
m = 0.5 #=> Massa do pêndulo
K = []
x =  abs(resultl.slope)
o = abs(resultt.slope)
for l in range(0, 10000):
    K.append(l/10000)
    I[l] = (-m*l/10000*x + o*m*(l/10000)**2)*(-o)**(-1)
item = 6500
aux = list(I)
print(K[item], I[item])
plt.plot(K, I, 'b')
plt.plot(K[item], I[item], 'or', label=f'Ponto escolhido, I = {np.round(I[item], 5)} $N\cdot m$ e $\ell$ = {K[item]} $m$')
plt.grid()
plt.title('Torque x Comprimento')
plt.xlabel('$\ell[m]$')
plt.ylabel('$I [N\cdot m]$')
plt.grid(False)
plt.legend()
plt.show()