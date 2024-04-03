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
env = gym.make("LunarLander-v2", render_mode="human")
#env = gym.make("LunarLander-v2", render_mode="human", enable_wind=True, wind_power=2.0, gravity=-11, turbulence_power=1)
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
    if ang_tgt >  0.5: ang_tgt =  0.5
    if ang_tgt < -0.5: ang_tgt = -0.5

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
    g = 0.2#(state[1]-state[3])
    if state[1]<0.65: g = (state[1]-state[3]+.975)#0.95
    if state[1]<0.05: g = (state[1]-state[3]+0.05)
    if alt_adj*g > np.abs(ang_adj) and alt_adj > 0.05: action = 2 #0.175
    elif ang_adj < -.05: action = 3#.25
    elif ang_adj > +.05: action = 1
    if sum(state[6:])==2: action = 0 #0.175
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
for t in range(3):
    observation = env.reset(seed=47+t)
    observation = observation[0]
    while 1:
        env.render()
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
            print("Episódio número: ", t+1) 
            g.append(cont)
            break

env.close()
fim = time.time()
# Calcula o tempo decorrido
tempo_decorrido = fim - inicio

print(f"Tempo decorrido: {tempo_decorrido} segundos")

print("Número de pousos: ", ooo)
d = 0
soma = np.zeros(3)
soma[0] = sum(r[:g[0]])
if soma[0]>=200:
    d = 1
for i in range(0, 2):
    soma[i+1] = sum(r[g[i]:g[i+1]])
    if sum(r[g[i]:g[i+1]])>200:
        d+=1

plt.plot(soma, 'ko',  label='Episódio')
print("Número de pousos com +200 pontos: ", d)
print("Média de pontuação: ", np.mean(soma))
plt.plot(np.ones(3)*200, 'r',  linewidth=2.5, label='+200 pontos')
plt.ylabel('Pontuação')
plt.xlabel('Episódios')
plt.title('Validação - PD')
plt.legend()
plt.show()
