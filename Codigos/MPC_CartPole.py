# Autor: Gabriel Bueno Leandro
# Título: Aplicação do PD

# Importando as bibliotecas
from itertools import product
import numpy as np
import matplotlib.pyplot as plt
import gym
import imageio
import time

Hp = 4 # Definindo o horizonte de controle, que é igual o de predição
p = np.array([-2, 0, 2]) # Opções de acordo com o delta u
b =product(p, repeat=Hp) # Aplicação do método product

Possibilidades = [] # lista fazia para receber todas as combinações
for i in b: 
    Possibilidades.append(i) # Salvar todas as possibilidades
Matriz = np.zeros((len(Possibilidades), Hp)) # Matriz para organizar todas as possibilidades
for i in range (0, len(Possibilidades)):
    Matriz[i, :] = np.array(Possibilidades[i])

action = 1 # Inicia a ação como sendo 1, força para direita
Soma = np.ones(np.shape(Matriz))*action # Matriz que auxilia na obtenção do delta u
for i in range(0, np.shape(Matriz)[0]): # Salva as possibilidadas + matriz soma
    for j in range(0, np.shape(Matriz)[1]): 
        Soma[i, j] = Matriz[i, j] + Soma[i, j-1]
a = []
for i in range(0, np.shape(Matriz)[0]): # Elimina resposta que não atendem ao requisitos estabelidos
    if max(Soma[i, :])>=2 or min(Soma[i, :])<=-2:
        a.append(i)
Soma_p = np.delete(Soma, a, 0) # Deleta todos os delta u inválidos início
Matriz_p = np.delete(Matriz, a, 0) # Matriz para organizar todas as possibilidades

def delu_p(action, Hp):
    """Obtendo delta u

    Args:
        action (int): acão
        Hp (int): horizonte de predição que é igual o horizonte de controle

    Returns:
        Matriz_p (ndarray): Matriz com todos os delta u possível para cada ação
    """
    p = np.array([-2, 0, 2]) # Opções de acordo com o delta u
    b =product(p, repeat=Hp) # Aplicação do método product
    Possibilidades = [] # lista fazia para receber todas as combinações
    for i in b:
        Possibilidades.append(i) # Salvar todas as possibilidades
    Matriz = np.zeros((len(Possibilidades), Hp)) # Matriz para organizar todas as possibilidades
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
    """Monta e retorna a matriz tau_a invertida

    Args:
        a1 (float): Termo modelo ARX y(k-1)
        a2 (float): Termo modelo ARX y(k-2)
        a3 (float): Termo modelo ARX y(k-2)
        N (int): Horizonte de controle

    Returns:
        ndarray : Matriz tau_a invertida
    """
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

def Sa(a1, a2, a3, N):
    """Monta e retorna a matriz sa

    Args:
        a1 (float): Termo modelo ARX y(k-1)
        a2 (float): Termo modelo ARX y(k-2)
        a3 (float): Termo modelo ARX y(k-2)
        N (int): Horizonte de controle

    Returns:
        ndarray : Matriz sa
    """
    Sa = np.zeros((N, 3))
    Sa[0, :] = a1, a2, a3
    Sa[1, 0:2] = a2, a3
    Sa[2, 0] = a3
    return Sa

def Tb(b1, b2, N):
    """Monta e retorna a matriz tau_b

    Args:
        b1 (float): Termo modelo ARX u(k-1)
        b2 (float): Termo modelo ARX u(k-2)
        N (int): Horizonte de predição

    Returns:
        ndarray: Matriz tau_b
    """
    b2 = 0
    Tb = np.zeros((N, N))
    for i in range(0, N):
        Tb[i, i] = b1
        if i+1<N:
            Tb[i+1, i] = b2
    return Tb

def predict(y, k, Matriz, action):
    """Aplicação do controle preditivo generalizado

    Args:
        y (ndarray): Array contendo os valores de ângulo
        k (int): Instante em ambiente se encontra, horizonte deslizante
        Matriz (ndarray): Matriz com todas as possibilidades de delta u
        action (int): Delta u inicial, o que é somado a ação

    Returns:
        int: Delta u inicial, o que é somado a ação
    """
    a1 = -3 # y(k-1)
    a2 = 2.99367 # y(k-2)
    a3 = -.99367 # y(k-3)
    b1 = -.00583 # u(k-1)
    b2 = 0 # u(k-2)
    Taa = Ta(a1, a2,a3, Hp) # Monta a matriz np.linalg.inv(tau_a)
    Tba = Tb(b1, b2, Hp) # Monta a matriz tau_b
    Saa = Sa(a1, a2,a3, Hp) # Monta a matriz s_a
    Matriz = delu_p(action, Hp) # Todas os deltas possíveis devido a ação
    yp = np.array([y[k], y[k-1], y[k-2]]).reshape(-1,1)
    soma = []
    G = Taa.dot(Tba)
    f = Taa.dot(Saa)
    for i in range(0, np.shape(Matriz)[0]):
        yo = (G.dot(Matriz[i, :].reshape(-1,1))-f.dot(yp))
        soma.append(float(abs(sum(yo))))
    return Matriz[soma.index(min(soma)),:]

env = gym.make("CartPole-v1", render_mode="human") # Inicio do ambiente e o seu modo de rederinzação
observation, info = env.reset(seed=42) # Reseta o ambiente e estabelece as condições iniciais
samples = 500 # Número de amostras 
y = np.zeros(samples) # Array para salvar o ângulo 
u = np.zeros(samples) # Array para salvar a entrada do CartPole
us = np.ones(samples) # Array para salvar a entrada da Função de Transferência
va = np.zeros(samples) # Array para salvar a velocidade angular
action = 1 # Definição ação inicial, pode ser 1 ou 0
c =[] # Lista para salvar o término de cada episódio
inicio = time.time() # Início da contagem de tempo
for i in range(samples): # Percorre todas as amostras
    observation, reward, terminated, truncated, info = env.step(action) # Aplica a ação
    u[i] = action # Salva a ação (CartPole) no array u
    y[i] = observation[2] # Salva o ângulo no array y
    va[i] = observation[1] # Salva a velocidade angular no array va
    if u[i]==0:
        us[i] = -1 # Adaptando a entrada para a Função de Transferência
    if i>1: # Verifica se há condições iniciais para chamar o GPC
        if action==0:
            action = -1
        a = predict(y, i, Matriz, action) # Calcula a ação devido ao GPC
    action = int(a[0]+action) # Soma o delta u a u
    if action==-1:
        action = 0
    if terminated or truncated:
        observation, info = env.reset() # Reseta o ambiente
        c.append(i) # Salva o amostra quando o episódio termina
env.close()
fime = time.time()
tempo_decorrido = fime - inicio # Calcula o tempo decorrido
print('tempo:', tempo_decorrido)
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