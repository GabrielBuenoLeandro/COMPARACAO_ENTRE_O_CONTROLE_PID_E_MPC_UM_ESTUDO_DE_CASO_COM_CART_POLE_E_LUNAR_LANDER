import gym
from matplotlib import pyplot as plt
import numpy as np
from time import sleep
import time
env = gym.make("CartPole-v1", render_mode='human')
observation = env.reset()
samples = 5000
Kp = 0.5
Ki = 0.25
Kd = 0.125

force = 0
integral = 0
y = np.zeros(samples)
u = np.zeros(samples)
va = np.zeros(samples)
inicio = time.time()
for _ in range(samples):
    env.render()

    observation, reward, done, info, aux = env.step(force)
    u[_] = force
    y[_] = observation[2]
    va[_] = observation[1]
    velocity = observation[1]
    angle = observation[2]
    angular_velocity = observation[3]

    integral = integral + angle

    F = Kp*(angle) + Kd*(angular_velocity) + Ki*(integral)

    force = 1 if F > 0 else 0
    if done:
        observation = env.reset()
        integral = 0
env.close()
fime = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fime - inic
print('tempo:', tempo_decorrido)
plt.figure(1)
plt.plot(y, 'g', label=r'$\theta$')
plt.plot(.2095*np.ones(len(u)),'r')
plt.plot(-.2095*np.ones(len(u)),'r', label='Limite')
plt.legend(bbox_to_anchor=(.875, .85), loc='center')
plt.ylabel(r'$\theta$ (rad)')
plt.xlabel('Amostras')
plt.title('Saída do Cartpole')

plt.show()