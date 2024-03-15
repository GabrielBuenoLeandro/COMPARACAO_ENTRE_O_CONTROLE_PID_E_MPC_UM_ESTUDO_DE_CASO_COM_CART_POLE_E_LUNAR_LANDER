import gym
from matplotlib import pyplot as plt

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