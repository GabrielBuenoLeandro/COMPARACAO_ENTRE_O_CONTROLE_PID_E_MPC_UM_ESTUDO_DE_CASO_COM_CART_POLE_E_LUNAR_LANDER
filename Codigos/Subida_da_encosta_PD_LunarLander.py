# Adaptação de "https://github.com/wfleshman/PID_Control", porém para o caso discreto do LunarLander
# Adptado por: Gabriel Bueno Leandro
# PID LunarLander Caso discreto

import gym
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
class Data():
    """Rastreia o estado"""
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
    
def pid(state, params):
    """Agoritmo controle PID

    Args:
        state (ndarray): Estado da sonda lunar adivindo do espaço de observação do Lunar Lander

    Returns:
        int: Saída do controlador PID
    """
    # Parâmetros do PID
    kp_alt = params[0]  # Proporcional da altitude
    kd_alt = params[1]  # Deritivo da altitude
    kp_ang = params[2]  # Proporcional do ângulo
    kd_ang = params[3]  # Derivativo do ângulo
    
    # Cálculo dos setpoints: ângulo e altitude
    alt_tgt = np.abs(state[0]) # |x| => módulo de x
    ang_tgt = (.25*np.pi)*(state[0]+state[2]) # A nave deve apontar em sentido ao objetivo
    if ang_tgt >  0.5: ang_tgt =  0.5 # Limitação de teto positivo
    if ang_tgt < -0.5: ang_tgt = -0.5 # Limitação de teto negativo

    # Cálculo dos erros
    alt_error = (alt_tgt - state[1]) # Erro de altitude
    ang_error = (ang_tgt - state[4]) # Erro angular
    
    # Use o PD para obter ajustes
    alt_adj = kp_alt*alt_error + kd_alt*state[3] # PD altitude
    ang_adj = kp_ang*ang_error + kd_ang*state[5] # PD ângulo
    if state[6] or state[7]: # Verifica se um dos pés está em contato com o solo
        ang_adj  = 0 # Se sim,  ang_adj é zero
        ang_adj = (state[3])*kd_alt # O multiplica pelo parâmetro derivativo da altitude
    action = 0 # Início, setando a ação em 0
    g = 0.2 # Inícia o múltiplo da altitude 
    if state[1]<0.65: g = (state[1]-state[3]+.975)# Se, y<0,65 o g asume valor maior
    if state[1]<0.05: g = (state[1]-state[3]+0.05)# Se, y<0,05 há uma desaceleração
    if alt_adj*g > np.abs(ang_adj) and alt_adj > 0.05: action = 2 # Verifica o acionamento do propulsor principal
    elif ang_adj < -.05: action = 3 # Verifica o acionamento do propulsor direito (auxiliar)
    elif ang_adj > +.05: action = 1 # Verifica o acionamento do propulsor esquerdo (auxiliar)
    if sum(state[6:])==2 and abs(state[0]<.125): action = 0  # Se ambos os pés estão no solo e a sonda está na plataforma
    return action # Retorna a ação


def run(params, env, verbose=False):
    """Executa um episódio dado parâmetros PID"""
    data = Data() 
    done = False
    state, info = env.reset()
    if verbose:
        env.render()
    data.add(state)
    total = 0
    while not done:
        a = pid(state,params)
        state,reward,done,_, cc = env.step(a)
        if verbose==True:
            rgb_array = env.render()
        total += reward
        if verbose:
            env.render()
        data.add(state)
    return total, data

def optimize(params, current_score, env, step):
    """Corre uma etapa de escalada aleatória"""

    # Adicionar ruído gaussiano (menos ruído à medida que n_steps aumenta)
    test_params = params + np.random.normal(0,20.0/step,size=params.shape)
    
    # Testa os parâmetros durante 20 episódios e tira a média
    scores = []
    for trial in range(20):
        score,_ = run(test_params,env)
        scores.append(score)
    avg = np.mean(scores)
    
    # Atualizar parâmetros se melhorados
    if avg > current_score:
        return test_params,avg
    else:
        return params,current_score
    
def main():
    # Ambiente LunarLander
    env = gym.make('LunarLander-v2', render_mode='rgb_array')
    env._max_episode_steps = 300
   
    # RNGs de sementes
    np.random.seed(0)

    # Escalada aleatória da subida sobre params
    params = np.array([0 , 0, 0, 0])
    score = -300 # Pior pontuação
    for steps in range(101):
        params,score = optimize(params,score,env,steps+1)
        if steps%10 == 0:
            print("Passo:",steps,"Pontuação:",score,"Parâmetros:",params)

    # Obter dados para execução final
    scores = []
    for trial in range(10):
        score, data = run(params, env, True)
        scores.append(score)
    env.close()
    print("Average Score:",np.mean(scores))
    data.graph()

if __name__ == '__main__':
    main()