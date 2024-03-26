# Autor: Gabriel Bueno Leandro
# Título: Aplicação do GPC

# Importando as bibliotecas
import numpy as np
import gym
import numpy as np
import matplotlib.pyplot as plt
import time
from itertools import product

# Como todos os modelos possuem a mesma estrutura: y(k) = a1y(k-1) + a2y(k-2) + b1u(k-1) + c, uma função será criada para montar as matrizes

def matriz(a1, a2, b1, hp):
    """Montagem das tatrizes: tau_a, tau_b e s_a, para isso suponha:

    y(k) = a1y(k-1) + a2y(k-2) + b1u(k-1) + c,

    para a formulação matricial, considera-se:

    y(k) - a1y(k-1) - a2y(k-2) = b1u(k-1) + c.

    Args:
        a1 (float): Parâmetro que acompanha o y(k-1), por formulação, deve o multiplicar por (-)
        a2 (float): Parâmetro que acompanha o y(k-2), por formulação, deve o multiplicar por (-)
        b1 (float): Parâmetro que acompanha o u(k-1)
        hp (int): Horizonte de predição, pode ir até 7 passos a frente

    Returns:
       ta (ndarray of floats): matriz tau_a
       sa (ndarray of floats): matriz s_a
       tb (ndarray of floats): matriz tau_b
    """
    if hp>7:
        hp = 7
    ta = np.array([[1, 0, 0, 0, 0, 0, 0],
                   [a1, 1, 0, 0, 0, 0, 0],
                   [a2, a1, 1, 0, 0, 0, 0],
                   [0, a2, a1, 1, 0, 0, 0],
                   [0, 0, a2, a1, 1, 0, 0],
                   [0, 0, 0, a2, a1, 1, 0],
                   [0, 0, 0, 0, a2, a1, 1]])
    sa = np.array([[a1, a2],
                   [a2, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0]])
    tb = np.array([[b1, 0, 0, 0, 0, 0, 0],
                   [0, b1, 0, 0, 0, 0, 0],
                   [0, 0, b1, 0, 0, 0, 0],
                   [0, 0, 0, b1, 0, 0, 0],
                   [0, 0, 0, 0, b1, 0, 0],
                   [0, 0, 0, 0, 0, b1, 0],
                   [0, 0, 0, 0, 0, 0, b1]])
    return np.linalg.inv(ta[:hp,:hp]),  sa[:hp,:hp],  tb[:hp,:hp]
# Horizonte de previsão
hp = 4
# Modelo posição x
a1x = -1.9986
a2x = 9.9860E-01
b1x = -7.3642E-05
cx = -4.7979E-04
taxi, sax, tbx = matriz(a1=a1x, a2=a2x, b1=b1x, hp=hp)
# Modelo posição y
a1y = -1.9928
a2y =  9.9279E-01
b1y =  8.1745E-05
cy = -3.9457E-04
tayi, say, tby = matriz(a1=a1y, a2=a2y, b1=b1y, hp=hp)
# Modelo para o ângulo da sonda
a1t = -1.9891
a2t = 9.8900E-01
b1t = 1.7201E-04
tati, sat, tbt = matriz(a1=a1t, a2=a2t, b1=b1t, hp=hp)
# Modelo para a velocidade linear em y
a1v = -1.2503
a2v = 2.5137E-01
b1v = -1.3791E-02
cv = 2.3644E-03 
tavi, sav, tbv = matriz(a1=a1v, a2=a2v, b1=b1v, hp=hp)
# Definindo o horizonte de controle, onde horizonte de controle < horizonte de predição(causalidaade)
hc = 2
# Opções de entrada: 0=>Não fazer nada, 1=>Orientação a esquerda, 2=>Propulsor principal e 3=>Orientação a direita
p = np.array([0, 1, 2, 3]) 
b =product(p, repeat=hc) # Aplicação do método product
Possibilidades = [] # lista fazia para receber todas as combinações
for i in b:
    Possibilidades.append(i)
x = np.zeros((len(Possibilidades), hc)) # Matriz para organizar todas as possibilidades
for i in range (0, len(Possibilidades)):
    x[i, :] = np.array(Possibilidades[i])
# Adição da entrada 0, como forma de preencher a  diferença entre o horizonte de predição e controle
H = np.zeros((int(4**hc), int(hp-hc)))
x = np.append(x, H, axis=1)

def MPC(px, py, k, at, v, params, pe):
    """Aplicação do GPC (Controle Preditivo)

    Args:
        px (ndarray): Entrada passadas de x até o instante k
        py (ndarray): Entrada passadas de y até o instante k
        k (int): Instante atual (k)
        at (ndarray): Entrada passadas de theta (ângulo theta) até o instante k
        v (ndarray):  Entrada passadas de vy até o instante k
        params (ndarray): Pesos calculados pelo algoritmo Subida de Encosta
        pe (ndarray): Vetor que checa se os pés da sonda está em contato com o solo lunar

    Returns:
        action (int): Ação que a sonda deve tomar (saída do controlador)
    """
    ypx = np.array([px[k], px[k-1]]).reshape(-1,1)
    ypy = np.array([py[k], py[k-1]]).reshape(-1,1)
    ypt = np.array([at[k], at[k-1]]).reshape(-1,1)
    ypv = np.array([v[k], v[k-1]]).reshape(-1,1)
    aux = np.ones(4)
    p = [-.725, -.125]
    p = np.poly1d(p)
    c_x = aux*cx
    c_y = aux*cy
    c_v = aux*cv
    ref = np.ones(1)*int(4**hc)
    s = np.ones(int(4**hc))*2000
    aux_x = -sax.dot(ypx)+c_x.reshape(-1,1)
    aux_y = -say.dot(ypy)+c_y.reshape(-1,1)
    aux_t = -sat.dot(ypt)
    aux_v = -say.dot(ypv)+c_v.reshape(-1,1)
    fx = taxi.dot(tbx)
    lx = taxi.dot(aux_x)
    fy = tayi.dot(tby)
    ly = tayi.dot(aux_y)
    ft = tati.dot(tbt)
    lt = tati.dot(aux_t)
    fv = tavi.dot(tbv)
    lv = tavi.dot(aux_v)
    for i in range(0, np.shape(x)[0]):
        u1 = np.zeros(4)
        u2 = np.zeros(4)
        for j in range(0, np.shape(x)[1]):
            if x[i, j] ==1:
                u2[j] = 1
                t = 1
            if x[i, j] ==3:
                u2[j] = -1
                t = 1
            if x[i, j] ==2:
                u1[j] = 1
                t = 1
        u1 = u1.reshape(-1, 1)
        u2 = u2.reshape(-1, 1)
        y_x = (fx.dot(u2)+lx)
        y_y = (fy.dot(u1)+ly)
        y_t = (ft.dot(u2)+lt)
        y_v = (fv.dot(u1)+lv)
        m = 0
        g = 1
        n = 1
        c = y_x
        if y_y[0]>y_x[0]:
            c = y_y
        ref = -p(np.abs(c))
        if y_v[0]<-ref[0]*.543:  
            m = params[0] 
            g = params[1] 
            n = params[2]  
        if np.sum(pe)==2:
            g=0
        s[i] = sum(abs(n*(y_x[:]-y_y[:])+m*(y_v[:]+ref[:])-g*(y_t[:])))
    s = list(s)
    u = s.index(min(s))
    action = x[u, 0]
    if sum(pe)!=0 and px[k]>-.1 and px[k]<.1:
        action = 0 
    return int(action)

params =  np.array([58.52230249, 23.10399406,  3.69236577])
frames = []
samples = 2000000
px = np.zeros(samples)
py = np.zeros(samples)
p1 = np.zeros(samples)
p2 = np.zeros(samples)
at = np.zeros(samples)
v = np.zeros(samples)
vx= np.zeros(samples)
vt = np.zeros(samples)
j = []
z = []
env = gym.make("LunarLander-v2", render_mode="rgb_array")
env.action_space.seed()
observation, info = env.reset()
r = observation[3]/observation[1]
action = 0
cont = 0
point = 0
g = []
inicio = time.time()
for i in range(samples):
    px[i] = observation[0]*(np.cos(np.pi/4))+observation[1]*(np.sin(np.pi/4))
    py[i] = observation[0]*(-np.sin(np.pi/4))+observation[1]*(np.cos(np.pi/4))
    at[i] = observation[4]
    v[i] = observation[3]
    vx[i] = observation[2]
    vt[i] = observation[5]
    if i>0:
        action = MPC(px, py, i, at, v, params, pe)
    observation, reward, terminated, truncated, info = env.step(action)
    pe = observation[6:]
    z.append(reward)
    if reward==100:
        cont+=1
    if terminated or truncated:
        observation, info = env.reset()
        j.append(reward)
        g.append(i)
        print(len(g))
        if len(g)>999:
            break
fim = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fim - inicio

print(f"Tempo decorrido: {tempo_decorrido} segundos")
env.close()
print(cont)