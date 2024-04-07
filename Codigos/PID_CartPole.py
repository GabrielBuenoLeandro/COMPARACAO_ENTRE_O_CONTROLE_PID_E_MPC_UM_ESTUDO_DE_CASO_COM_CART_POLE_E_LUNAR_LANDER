# Autor: Gabriel Bueno Leandro
# Título: Aplicação do PD

# Importando as bibliotecas
import gym
from matplotlib import pyplot as plt
import numpy as np
from time import sleep
import time

env = gym.make("CartPole-v1", render_mode='human') # Importando o ambiente CartPole
observation = env.reset(seed=42) # Reinicia o ambiente de simulação 
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
    env.render() # Renderiza o ambiente
    observation, reward, done, info, aux = env.step(force) # Aplica a entrada ao sistema
    u[_] = force # Salva a entrada no vetor u
    y[_] = observation[2] # Salva o ângulo no vetor y
    va[_] = observation[1] # Salva a velocidade angular no vetor va
    velocity = observation[1] # Salva a velocidade no int velocity
    angle = observation[2] # Salva o ângulo no int angle
    angular_velocity = observation[3] # Salva a velocidade ângular no int angular_velocity
    integral = integral + angle # Termo integral, para isso usa um somatório
    F = Kp*(angle) + Kd*(angular_velocity) + Ki*(integral) # Aplicação do PID
    force = 1 if F > 0 else 0 # Se F é positivo force recebe 1, se negativo F = 0
    if done: # Verifica o término
        observation = env.reset() # Reseta o ambiente
        integral = 0 # A cada novo episódio, o termo integral deve zerar
env.close() # Encerra o ambiente
fime = time.time() # Encerra a contagem de tempo
tempo_decorrido = fime - inicio # Calcula o tempo decorrido
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