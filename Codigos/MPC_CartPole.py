from itertools import product
import numpy as np
import matplotlib.pyplot as plt
import gym
import imageio
import time

Hp = 4
# Opções de acordo com o delta u
p = np.array([-2, 0, 2]) 
b =product(p, repeat=Hp) # Aplicação do método product

Possibilidades = [] # lista fazia para receber todas as combinações
for i in b:
    Possibilidades.append(i)
Matriz = np.zeros((len(Possibilidades), Hp)) # Matriz para organizar todas as possibilidades
for i in range (0, len(Possibilidades)):
    Matriz[i, :] = np.array(Possibilidades[i])

action = 1
Soma = np.ones(np.shape(Matriz))*action
for i in range(0, np.shape(Matriz)[0]):
    for j in range(0, np.shape(Matriz)[1]):
        Soma[i, j] = Matriz[i, j] + Soma[i, j-1]
a = []
for i in range(0, np.shape(Matriz)[0]):
    if max(Soma[i, :])>=2 or min(Soma[i, :])<=-2:
        a.append(i)
Soma_p = np.delete(Soma, a, 0)
Matriz_p = np.delete(Matriz, a, 0)

def delu_p(action, Hp):
    p = np.array([-2, 0, 2]) # 2 <=> 3
    b =product(p, repeat=Hp)
    
    Possibilidades = [] # lista fazia para receber todas as combinações
    for i in b:
        Possibilidades.append(i)
    Matriz = np.zeros((len(Possibilidades), Hp))
    for i in range (0, len(Possibilidades)):
        Matriz[i, :] = np.array(Possibilidades[i])
    Soma = np.ones(np.shape(Matriz))*action
    for i in range(0, np.shape(Matriz)[0]):
        for j in range(0, np.shape(Matriz)[1]):
            Soma[i, j] = Matriz[i, j] + Soma[i, j-1]
    a = []
    for i in range(0, np.shape(Matriz)[0]):
        if max(Soma[i, :])>=2 or min(Soma[i, :])<=-2:
            a.append(i)
    Soma_p = np.delete(Soma, a, 0)
    Matriz_p = np.delete(Matriz, a, 0)
    return Matriz_p

def Ta(a1, a2, a3, N):
    Ta = np.zeros((N, N))
    for i in range(0, N):
        Ta[i, i] = 1
        if i+1<N:
            Ta[i+1, i] = a1
        if i+2<N:
            Ta[i+2, i] = a2
        if i+3<N:
            Ta[i+3, i] = a3
    return np.linalg.inv(Ta)

def Sa1(a1, a2, a3, N):
    return np.array([[a1, a2, a3]])

def Sa(a1, a2, a3, N):
    Sa = np.zeros((N, 3))
    Sa[0, :] = a1, a2, a3
    Sa[1, 0:2] = a2, a3
    Sa[2, 0] = a3
    return Sa

def Tb(b1, b2, N):
    b2 = 0
    Tb = np.zeros((N, N))
    for i in range(0, N):
        Tb[i, i] = b1
        if i+1<N:
            Tb[i+1, i] = b2
    return Tb

def predict(y, k, Matriz, action):
    ymin = 10
    a1 = -3
    a2 = 2.99367
    a3 = -.99367
    b1 = -0.00583
    b2 = 0
    N = 4
    Taa = Ta(a1, a2,a3, N)
    Tba = Tb(b1, b2, N)
    Saa = Sa(a1, a2,a3, N)
    Matriz = delu_p(action, N)
    yp = np.array([y[k], y[k-1], y[k-2]]).reshape(-1,1)
    soma = []
    G = Taa.dot(Tba)
    f = Taa.dot(Saa)
    for i in range(0, np.shape(Matriz)[0]):
        yo = (G.dot(Matriz[i, :].reshape(-1,1))-f.dot(yp))
        soma.append(float(abs(sum(yo))))
    return Matriz[soma.index(min(soma)),:]

env = gym.make("CartPole-v1", render_mode="human")
observation, info = env.reset()
samples = 500
y = np.zeros(samples)
u = np.zeros(samples)
us = np.ones(samples)
va = np.zeros(samples)
action = 1
frames = []
c =[]
inicio = time.time()
for i in range(samples):
    observation, reward, terminated, truncated, info = env.step(action)
    rgb_array = env.render()
    frames.append(rgb_array)
    u[i] = action
    y[i] = observation[2]
    va[i] = observation[1]
    if u[i]==0:
        us[i] = -1
    #y[i] = observation[2]
    if i>1:
        if action==0:
            action = -1
        a = predict(y, i, Matriz, action)
    action = int(a[0]+action)
    if action==-1:
        action = 0
    #u[i] = action
    #y[i] = observation[2]
    if terminated or truncated:
        observation, info = env.reset()
        c.append(i)
env.close()
fime = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fime - inicio
print('tempo:', tempo_decorrido)
#imageio.mimsave('ctmpc.gif', frames, duration=0.1)

plt.plot(y,'k', label='$\\theta$')
plt.plot(np.ones(len(y))*.2095, 'r', label='Limite')
plt.plot(np.ones(len(y))*-.2095,'r')
plt.legend()
plt.yticks([-0.2, -0.1, 0, 0.1, 0.2])
plt.ylim(-.25, .25)
plt.legend(bbox_to_anchor=(0.85, 0.82), loc='center')
plt.xlabel('Amostras')
plt.ylabel('$\\theta (rad)$')
plt.title('Saída do Cartpole - GPC')
plt.show()