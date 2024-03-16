import numpy as np
import gym
import time
import matplotlib.pyplot as plt

env = gym.make("CartPole-v1", render_mode="rgb_array")
observation, info = env.reset()

U = [] # Lista para salvar as entradas
Y = []
T = []
P = []

for _ in range(1000):
    start_time = time.perf_counter()
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)
    U.append(action)
    Y.append(observation[2])
    P.append(action)
    
    if terminated or truncated:
        end_time = time.perf_counter()
        observation, info = env.reset()
        T.append(_)
        elapsed_time = end_time - start_time

        #print(f"Tempo decorrido: {elapsed_time} segundos")

env.close()

t = np.linspace(0, 1000, 1000)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, gridspec_kw={'hspace': 0.32})
# Plotando as duas figuras nos subplots
ax1.plot(t, Y, 'r', label='Saída')
for k in range (0, len(T)):
    i = k-1
    ax2.plot(t[T[i]+1:T[i+1]+1],Y[T[i]+1:T[i+1]+1], 'b')
ax2.plot(t[0:T[0]+1],Y[0:T[0]+1], 'b')
ax1.set_ylabel('$\\theta (rad)$')
ax2.set_ylabel('$\\theta (rad)$')
ax1.set_xlabel('Amostras')
ax1.set_title('Saída')
ax2.set_title('Episódios Completos')
ax2.set_xlabel('Amostras')