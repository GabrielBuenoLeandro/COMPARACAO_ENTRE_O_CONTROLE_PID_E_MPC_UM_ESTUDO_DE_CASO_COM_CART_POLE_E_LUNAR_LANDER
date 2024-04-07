# Autor: Gabriel Bueno Leandro
# Título: Aplicação do PD

# Importando as bibliotecas
import gym
from matplotlib import pyplot as plt
import numpy as np
from time import sleep
import time

env = gym.make("CartPole-v1", render_mode='human') # Importando o ambiente CartPole
observation = env.reset() # Reinicia o ambiente de simulação 
samples = 500 # Selecione o número de amostras
Kp = 0.5 # Termo Proporcional
Ki = 0.25 # Termo Integral
Kd = 0.125 # Termo Derivativo
force = 0 # Entrada do sistema, aqui definida como força
integral = 0 # O termo que receberá o somatório do termo integral
y = np.zeros(samples) # Array para salvar o ângulo do pêndulo
u = np.zeros(samples) # Array para salvar a ação/entrada do pêndulo
va = np.zeros(samples) # Array para salvar a velocidade angular do pêndulo
inicio = time.time() # Início da contagem do tempo 
for _ in range(samples): # Percorre todos os episódios
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
tempo_decorrido = fime - inicio
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