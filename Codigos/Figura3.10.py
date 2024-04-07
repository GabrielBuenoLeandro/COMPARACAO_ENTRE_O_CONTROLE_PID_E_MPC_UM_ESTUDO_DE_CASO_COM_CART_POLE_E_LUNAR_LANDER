import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import gym
import time
from sysidentpy.metrics import root_relative_squared_error
env = gym.make("CartPole-v1", render_mode="rgb_array")
observation, info = env.reset()

U = [] # Lista para salvar as entradas
Y = []
T = []
P = []

for _ in range(1000):
    start_time = time.perf_counter()
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)
    U.append(action)
    Y.append(observation[2])
    P.append(action)
    
    if terminated or truncated:
        end_time = time.perf_counter()
        observation, info = env.reset()
        T.append(_)
        elapsed_time = end_time - start_time

        #print(f"Tempo decorrido: {elapsed_time} segundos")

env.close()


for i in range (0, len(U)):
    if U[i]==0:
        U[i]=-1
    else:
        U[i] = 1

# Pesos diferentes:
xi = 0.19524143238862357
oi = -0.29775850872198745
u = 1
l = 0.65
m = 0.5
g = 9.81
I = 0.0018537860132090966
erro = np.zeros(1146)
ka = []
t = np.linspace(0, 10, 1000)  # Cria um vetor de tempo de 0 a 10 segundos
for k in range(1, 1147):
    x = xi*k
    o = oi*k
    ka.append(k)
    M = (1-m*x-m*l*o)/x
    a = m*l
    b = ((m**2*l**2-(M+m)*(I+m*l**2)))
    c = ((M+m)*m*g*l)
    numerator = [a]
    denominator = [b, 0, c]  # Exemplo: Sistema de segunda ordem
    H = signal.TransferFunction(numerator, denominator)
    for p in range(0, (len(T)-1)):
        if p==0 and k==1:
            print(a)
        if p ==0:
            ttf, y, x = signal.lsim(H, U=U[:T[0]+1], T=t[:T[0]+1])
            er = root_relative_squared_error(y+Y[0], Y[:T[0]+1])
        else:
            i = p-1
            #print(i)
            ttf, y, x = signal.lsim(H, U=U[T[i]+1:T[i+1]+1], T=t[T[i]+1:T[i+1]+1])
            erro[k-1] = erro[k-1] + root_relative_squared_error(y+Y[T[i]+1], Y[T[i]+1:T[i+1]+1])
        erro[k-1] = erro[k-1] + er

print('Episódios completos: ', len(T))
        
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, gridspec_kw={'hspace': 0.32})
# Plotando as duas figuras nos subplots
aux = list(erro)
posmin = aux.index(min(aux))
erro = np.array(erro)
ax1.plot(erro[:]/len(T), 'b')
ax2.plot(np.arange(posmin-20, posmin+20), erro[posmin-20:posmin+20]/len(T), 'b')
ax2.plot(posmin, min(erro)/len(T),'go', label='$\\overline{RMSE}$ Mínimo')
ax2.legend()
ax1.set_ylabel('$\\overline{RMSE}$')
ax2.set_ylabel('$\\overline{RMSE}$')
ax1.set_xlabel('Valor de k')
ax1.set_title('Validação da FT')
ax2.set_title('Validação da FT: Um olhar de perto')
ax2.set_xlabel('Valor de k')