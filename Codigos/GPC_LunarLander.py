import numpy as np
import imageio
import gym
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

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

frames = []
class Data():
    """tracks elements of the state"""
    def __init__(self):
        self.states = []
    
    def add(self,state):
        self.states.append(state)
        
    def graph(self):
        states = np.array(self.states).reshape(len(self.states),-1)
        plt.plot(states[:,0],label='x')
        plt.plot(states[:,1],label='y')
        plt.plot(states[:,2],label='vx')
        plt.plot(states[:,3],label='vy')
        plt.plot(states[:,4],label='theta')
        plt.plot(states[:,5],label='vtheta')
        plt.legend()
        plt.grid()
        plt.ylim(-1.1,1.1)
        plt.title('Controle PID')
        plt.ylabel('Valor')
        plt.xlabel('Amostras')
        plt.show('pid.png')


def run(params, env, verbose=False):
    """ runs an episode given pid parameters """
    data = Data() 
    done = False
    state, info = env.reset()
    i = 0
    pe  = np.zeros(2)
    px = []
    py = []
    p1 = []
    p2 = []
    at = []
    v = []
    vx= []
    vt = []
    if verbose:
        env.render()
        sleep(.005)
    data.add(state)
    total = 0
    while not done and len(p2)<env._max_episode_steps:
        observation = state
        px.append(observation[0]*(np.cos(np.pi/4))+observation[1]*(np.sin(np.pi/4)))
        py.append(observation[0]*(-np.sin(np.pi/4))+observation[1]*(np.cos(np.pi/4)))
        at.append(observation[4])
        v.append(observation[3])
        vx.append(observation[2])
        vt.append(observation[5])
        pe = np.copy(observation[6:])
        a = 0
        if i>=2:
            a = MPC(px, py, i, p1, p2, at, v, params, pe)
        if a == 1: 
            p2.append(1)
            p1.append(0)
        if a == 3: 
            p2.append(-1)
            p1.append(0)
        if a == 2:
            p1.append(1)
            p2.append(0)
        state,reward,done,_, cc = env.step(a)
        i +=1
        if verbose==True:
            rgb_array = env.render()
            #frames.append(rgb_array)
        total += reward
        if verbose:
            env.render()
            sleep(.005)
        data.add(state)
    return total, data

def optimize(params, current_score, env, step):
    """ runs a step of randomized hill climbing """

    # add gaussian noise (less noise as n_steps increases)
    test_params = params + np.random.normal(0,20.0/step,size=params.shape)
    
    # test params over 5 trial avg
    scores = []
    for trial in range(5):
        score,_ = run(test_params,env)
        scores.append(score)
    avg = np.mean(scores)
    
    # update params if improved
    if avg > current_score:
        return test_params,avg
    else:
        return params,current_score
    
def main():
    # Setup environment
    env = gym.make('LunarLander-v2', render_mode='rgb_array')
    env._max_episode_steps = 450
   
    # Seed RNGs
    np.random.seed(0)
    #env.seed(0)

    # Random Hill Climb over params
    params = np.array([0, 0, 0, 0, 0])
    score = -300 # bad starting score
    for steps in range(101):
        params,score = optimize(params,score,env,steps+1)
        if steps%10 == 0:
            print("Step:",steps,"Score:",score,"Params:",params)

    # Get data for final run
    scores = []
    for trial in range(10):
        score, data = run(params, env, True)
        scores.append(score)
    env.close()
    print("Average Score:",np.mean(scores))
    data.graph()

if __name__ == '__main__':
    main()

params = np.array([24.76809389, 23.43504149, 20.08773842, 48.05462707, 42.97594957])
frames = []
samples = 2000
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
g = []
pe = np.zeros(2)
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
env.close()


plt.plot(px,label='$x$')
plt.plot(py,label='$y$')
plt.plot(vx,label='$v_x$')
plt.plot(v,label='$v_y$')
plt.plot(at,label='$\\theta$')
plt.plot(vt,label='$v_\\theta$')
plt.legend()
plt.grid()
plt.ylim(-1.1,1.1)
plt.title('Controle PID')
plt.ylabel('Valor')
plt.xlabel('Amostras')