# Autor: Gabriel Bueno Leandro
# Título: Aplicação do PD

# Importando as bibliotecas
import gym
import tempfile
import numpy as np
import time
import matplotlib.pyplot as plt
# Definindo o ambiente
tdir = tempfile.mkdtemp()
env = gym.make('LunarLander-v2', render_mode='human')
print (env.action_space)
p = []

def pid(state):
    """ calculates settings based on pid control """
    # PID parameters
    kp_alt = 9.0565  # proportional altitude
    kd_alt = -9.9488  # derivative altitude
    kp_ang = 11.9271  # proportional angle
    kd_ang = -5.0963  # derivative angle
    
    # Calculate setpoints (target values)
    alt_tgt = np.abs(state[0])
    ang_tgt = (.25*np.pi)*(state[0]+state[2])
    if ang_tgt >  0.785: ang_tgt =  0.785
    if ang_tgt < -0.785: ang_tgt = -0.785

    # Calculate error values
    alt_error = (alt_tgt - state[1])
    ang_error = (ang_tgt - state[4])
    
    # Use PID to get adjustments
    alt_adj = kp_alt*alt_error + kd_alt*state[3]
    ang_adj = kp_ang*ang_error + kd_ang*state[5]
    if state[6] or state[7]:
        ang_adj  = 0
        ang_adj = (state[3])*kd_alt
    action = 0
    if alt_adj > 0.5*np.abs(ang_adj) and alt_adj > 0.2: action = 2 #0.175
    elif ang_adj < -.1: action = 3#.25
    elif ang_adj > +.1: action = 1
    return action

r = []
x = []
y = []
vx = []
vy = []
th = []
vt = []
g = []
cont = 0
ooo = 0
inicio = time.time()
# 100 trials for landing
for t in range(1000):
    observation = env.reset()
    observation = observation[0]
    while 1:
        #env.render()
        cont+=1
        #print(observation)
        # select action using pid method
        action = pid(observation)
        observation, reward, done, info, a = env.step(action)
        r.append(reward)
        if reward==100:
            ooo+=1
        x.append(observation[0])
        y.append(observation[1])
        vx.append(observation[2])
        vy.append(observation[3])
        th.append(observation[4])
        vt.append(observation[5])
        if done:
            print("Episode finished after {} timesteps".format(t))
            g.append(cont)
            print(cont)
            break

env.close()
fim = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fim - inicio

print(f"Tempo decorrido: {tempo_decorrido} segundos")

print(ooo)
d = 0
soma = np.zeros(10)
soma[0] = sum(r[:g[0]])
plt.plot(0, sum(r[:g[0]]), 'or--')
for i in range(0, 9):
    plt.plot(i+1, sum(r[g[i]:g[i+1]]), 'o--')
    soma[i+1] = sum(r[g[i]:g[i+1]])
    if sum(r[g[i]:g[i+1]])>200:
        d+=1
print(d)