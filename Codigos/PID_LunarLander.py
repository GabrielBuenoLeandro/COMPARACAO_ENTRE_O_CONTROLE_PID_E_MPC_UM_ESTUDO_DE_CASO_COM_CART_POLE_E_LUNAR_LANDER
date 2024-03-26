import gym
import tempfile
import numpy as np
import time
states=[]

from gym import wrappers

tdir = tempfile.mkdtemp()
env = gym.make('LunarLander-v2', render_mode='rgb_array')
print (env.action_space)
p = []

def pid(state, params):
    """ calculates settings based on pid control """
    # PID parameters
    kp_alt = params[0]  # proportional altitude
    kd_alt = params[1]  # derivative altitude
    kp_ang = params[2]  # proportional angle
    kd_ang = params[3]  # derivative angle
    
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
        ang_adj = -(state[3])*0.5
        
    action = 0
    if alt_adj > np.abs(ang_adj) and alt_adj > 1.45: action = 2
    elif ang_adj < -0.05: action = 3
    elif ang_adj > +0.05: action = 1
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
params = np.array([9.0565, -9.9488, 11.9271, -5.0963])
inicio = time.time()
# 100 trials for landing
for t in range(1000):
    observation = env.reset()
    observation = observation[0]
    while 1:
        env.render()
        cont+=1
        #print(observation)
        # select action using pid method
        action = pid(observation, params)
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