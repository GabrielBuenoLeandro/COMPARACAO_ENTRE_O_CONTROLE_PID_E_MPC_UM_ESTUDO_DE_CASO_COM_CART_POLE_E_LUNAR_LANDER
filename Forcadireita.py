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
# Plote dos resultados
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
ax1.plot(P[:t[0]])
ax1.set_xlabel('Amostras')
ax1.set_ylabel('m')
ax1.set_title("Posição")
ax2.plot(V[:t[0]], 'tab:orange')
ax2.set_xlabel('Amostras')
ax2.set_ylabel('m/s')
ax2.set_title("Velocidade")
ax3.plot(A[:t[0]], 'tab:green')
ax3.set_xlabel('Amostras')
ax3.set_ylabel('rad')
ax3.set_title("Ângulo")
ax4.plot(Va[:t[0]], 'tab:red')
ax4.set_xlabel('Amostras')
ax4.set_ylabel('rad/s')
ax4.set_title("Velocidade angular")
fig.tight_layout()
plt.show()

# Acelerações linear e angular
x = np.arange(0, t[0])
Vl = np.array(V[:t[0]])
Vt = np.array(Va[:t[0]])

# Aplicando a regressão linear
resultl = linregress(x, Vl)
resultt = linregress(x, Vt)

# Exibindo os resultados
print("Aceleração Linear")
print("Inclinação:", resultl.slope)
print("Interceptação:", resultl.intercept)
print("Coeficiente de correlação:", resultl.rvalue)
print("Valor p:", resultl.pvalue)
print("Erro padrão:", resultl.stderr)
print()
print("Aceleração Angular")
print("Inclinação:", resultt.slope)
print("Interceptação:", resultt.intercept)
print("Coeficiente de correlação:", resultt.rvalue)
print("Valor p:", resultt.pvalue)
print("Erro padrão:", resultt.stderr)

# Gráfico: "Relação entre torque e comprimento, para $m=0,5$,"
I = np.zeros(10000)
m = 0.5
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