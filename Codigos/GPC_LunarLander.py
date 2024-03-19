import numpy as np
import gym
import time

a1 = -1.9986
a2 = 9.9860E-01
b1 = -7.3642E-05
cx = -4.7979E-04

tax = np.array([[1, 0, 0, 0, 0, 0, 0],
               [a1, 1, 0, 0, 0, 0, 0],
               [a2, a1, 1, 0, 0, 0, 0],
               [0, a2, a1, 1, 0, 0, 0],
               [0, 0, a2, a1, 1, 0, 0],
               [0, 0, 0, a2, a1, 1, 0],
               [0, 0, 0, 0, a2, a1, 1]])

sax = np.array([[a1, a2],
               [a2, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0]])

tbx = np.array([[b1, 0, 0, 0, 0, 0, 0],
                [0, b1, 0, 0, 0, 0, 0],
                [0, 0, b1, 0, 0, 0, 0],
                [0, 0, 0, b1, 0, 0, 0],
                [0, 0, 0, 0, b1, 0, 0],
                [0, 0, 0, 0, 0, b1, 0],
                [0, 0, 0, 0, 0, 0, b1]])

tax = tax[:,:]
sax = sax[:,:]
tbx = tbx[:,:]

a1 = -1.9928
a2 = 9.9279E-01
b1 = 8.1745E-04
cy = -3.9457E-04


tay = np.array([[1, 0, 0, 0, 0, 0, 0],
               [a1, 1, 0, 0, 0, 0, 0],
               [a2, a1, 1, 0, 0, 0, 0],
               [0, a2, a1, 1, 0, 0, 0],
               [0, 0, a2, a1, 1, 0, 0],
               [0, 0, 0, a2, a1, 1, 0],
               [0, 0, 0, 0, a2, a1, 1]])

say = np.array([[a1, a2],
               [a2, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0]])

tby = np.array([[b1, 0, 0, 0, 0, 0, 0],
                [0, b1, 0, 0, 0, 0, 0],
                [0, 0, b1, 0, 0, 0, 0],
                [0, 0, 0, b1, 0, 0, 0],
                [0, 0, 0, 0, b1, 0, 0],
                [0, 0, 0, 0, 0, b1, 0],
                [0, 0, 0, 0, 0, 0, b1]])

tay = tay[:,:]
say = say[:,:]
tby = tby[:,:]

a1 = -1.9891
a2 = 9.8900E-01
b1 = 1.7201E-04


tat = np.array([[1, 0, 0, 0, 0, 0, 0],
               [a1, 1, 0, 0, 0, 0, 0],
               [a2, a1, 1, 0, 0, 0, 0],
               [0, a2, a1, 1, 0, 0, 0],
               [0, 0, a2, a1, 1, 0, 0],
               [0, 0, 0, a2, a1, 1, 0],
               [0, 0, 0, 0, a2, a1, 1]])

sat = np.array([[a1, a2],
               [a2, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0]])

tbt = np.array([[b1, 0, 0, 0, 0, 0, 0],
                [0, b1, 0, 0, 0, 0, 0],
                [0, 0, b1, 0, 0, 0, 0],
                [0, 0, 0, b1, 0, 0, 0],
                [0, 0, 0, 0, b1, 0, 0],
                [0, 0, 0, 0, 0, b1, 0],
                [0, 0, 0, 0, 0, 0, b1]])

tat = tat[:,:]
sat = sat[:,:]
tbt = tbt[:,:]

a1 = -5.5667E-01
a2 = -3.2378E-01 
b1 = 2.1617E-02
cv = -7.8055E-02


tav = np.array([[1, 0, 0, 0, 0, 0, 0],
               [a1, 1, 0, 0, 0, 0, 0],
               [a2, a1, 1, 0, 0, 0, 0],
               [0, a2, a1, 1, 0, 0, 0],
               [0, 0, a2, a1, 1, 0, 0],
               [0, 0, 0, a2, a1, 1, 0],
               [0, 0, 0, 0, a2, a1, 1]])

sav = np.array([[a1, a2],
               [a2, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0],
               [0, 0]])

tbv = np.array([[b1, 0, 0, 0, 0, 0, 0],
                [0, b1, 0, 0, 0, 0, 0],
                [0, 0, b1, 0, 0, 0, 0],
                [0, 0, 0, b1, 0, 0, 0],
                [0, 0, 0, 0, b1, 0, 0],
                [0, 0, 0, 0, 0, b1, 0],
                [0, 0, 0, 0, 0, 0, b1]])

c = 4
tav = tav[:,:]
sav = sav[:,:]
tbv = tbv[:,:]

from itertools import product

Hc = c
# Opções de acordo com o delta u
p = np.array([0, 1, 2, 3]) 
b =product(p, repeat=Hc) # Aplicação do método product

Possibilidades = [] # lista fazia para receber todas as combinações
for i in b:
    Possibilidades.append(i)
x = np.zeros((len(Possibilidades), Hc)) # Matriz para organizar todas as possibilidades
for i in range (0, len(Possibilidades)):
    x[i, :] = np.array(Possibilidades[i])
    
H = np.zeros((256, 3))
x = np.append(x, H, axis=1)
print(np.shape(x))

def MPC(px, py, k, p1, p2, at, v, params, pe):
    ypx = np.array([px[k], px[k-1]]).reshape(-1,1)
    ypy = np.array([py[k], py[k-1]]).reshape(-1,1)
    ypt = np.array([at[k], at[k-1]]).reshape(-1,1)
    ypv = np.array([v[k], v[k-1]]).reshape(-1,1)
    aux = np.ones(7)
    p = [-.725, -.125]
    p = np.poly1d(p)
    c_x = aux*cx
    c_y = aux*cy
    c_v = aux*cv
    ref = np.ones(1)*256
    s = np.ones(256)*2000
    tavi = np.linalg.inv(tax)
    taxi = np.linalg.inv(tax)
    tati = np.linalg.inv(tat)
    tayi = np.linalg.inv(tay)
    aux_x = -sax.dot(ypx)+c_x.reshape(-1,1)
    aux_y = -say.dot(ypy)+c_y.reshape(-1,1)
    aux_t = -sat.dot(ypt)
    aux_v = -say.dot(ypv)+c_v.reshape(-1,1)
    for i in range(0, np.shape(x)[0]):
        u1 = np.zeros(7)
        u2 = np.zeros(7)
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
        y_x = taxi.dot(tbx.dot(u2)+aux_x)
        y_t = tati.dot(tbt.dot(u2)+aux_t)
        y_y = tayi.dot(tbx.dot(u1)+aux_y)
        y_v = tavi.dot(tbv.dot(u1)+aux_v)
        m = 0
        g = params[4]
        n = params[3]
        ref = -p(y_y)
        if y_v[0]<-ref[0]:  #.95
            m = params[0] #0.0145
            g = params[1] #0.7
            n = params[2]  #0.55
        s[i] = sum(abs(1*n*(y_x[:]-y_y[:])+m*(y_v[:]+ref[:])-(g*y_t[:])))
    s = list(s)
    u = s.index(min(s))
    action = x[u, 0]
    if sum(pe)!=0 and px[k]>-.1 and px[k]<.1:
        action = 0 
    return int(action)

params = np.array([10.44842328, 20.27103112, 20.89498623, 45.92627943, 55.43402588])
frames = []
samples = 200000
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
env = gym.make("LunarLander-v2", render_mode="human")
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
    if i>=2:
        action = MPC(px, py, i, p1, p2, at, v, params, pe)
    if action == 1: 
        p2[i] = 1
    if action == 3: 
        p2[i] = -1
    if action == 2:
        p1[i] = 1
    observation, reward, terminated, truncated, info = env.step(action)
    pe = observation[6:]
    z.append(reward)
    if reward==100:
        cont+=1
    if terminated or truncated:
        observation, info = env.reset()
        j.append(reward)
        g.append(i)
        if len(g)>19:
            break
fim = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fim - inicio

print(f"Tempo decorrido: {tempo_decorrido} segundos")
env.close()
print(cont)