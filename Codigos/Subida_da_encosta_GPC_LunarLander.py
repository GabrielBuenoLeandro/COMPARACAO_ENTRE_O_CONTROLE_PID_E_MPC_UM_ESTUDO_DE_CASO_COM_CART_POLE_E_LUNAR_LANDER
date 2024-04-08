# Adaptação de "https://github.com/wfleshman/PID_Control", porém para o caso discreto do LunarLander
# Adptado por: Gabriel Bueno Leandro
# GPC LunarLander Caso discreto

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
       ndarray of floats: matriz tau_a
       ndarray of floats: matriz s_a
       ndarray of floats: matriz tau_b
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

def MPC(px, py, k, p1, p2, at, v, params, pe):
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
        int: Ação que a sonda deve tomar (saída do controlador)
    """
    ypx = np.array([px[k], px[k-1]]).reshape(-1,1) # Coleta de p_x(k) e p_x(k-1)
    ypy = np.array([py[k], py[k-1]]).reshape(-1,1) # Coleta de p_y(k) e p_y(k-1)
    ypt = np.array([at[k], at[k-1]]).reshape(-1,1) # Coleta de \theta(k) e \theta(k-1)
    ypv = np.array([v[k], v[k-1]]).reshape(-1,1) # Coleta de v_y(k) e v_y(k-1)
    aux = np.ones(hp)
    p = [-.725, -.125] # Setpint para velocidade linear em y (-0,725*p_y - 0.125)
    p = np.poly1d(p) # Cria polinômio no objeto p (para calcular o setpoint)
    c_x = aux*cx # Termo constante p_x com dimenção (H_p x 1)
    c_y = aux*cy # Termo constante p_y com dimenção (H_p x 1)
    c_v = aux*cv # Termo constante v_y com dimenção (H_p x 1)
    s = np.ones(int(4**hc))*2000 # Vetor para salvar os valores da função custo, como o valor almejado é o mínimo, salva um valor grande em todo vetor
    aux_x = -sax.dot(ypx)+c_x.reshape(-1,1) # Cálculo de s_a * p_x mais o termo constante
    aux_y = -say.dot(ypy)+c_y.reshape(-1,1) # Cálculo de s_a * p_y mais o termo constante
    aux_t = -sat.dot(ypt)  # Cálculo de s_a * \theta
    aux_v = -say.dot(ypv)+c_v.reshape(-1,1)  # Cálculo de s_a * v_y mais o termo constante
    fx = taxi.dot(tbx)  # Matriz G, para obtenção da resposta forçada de p_x
    lx = taxi.dot(aux_x) # Resposta livre para p_x
    fy = tayi.dot(tby) # Matriz G, para obtenção da resposta forçada de p_y
    ly = tayi.dot(aux_y) # Resposta livre para p_y
    ft = tati.dot(tbt) # Matriz G, para obtenção da resposta forçada de \theta
    lt = tati.dot(aux_t) # Resposta livre para \theta
    fv = tavi.dot(tbv) # Matriz G, para obtenção da resposta forçada de v_y
    lv = tavi.dot(aux_v) # Resposta livre para v_y
    for i in range(0, np.shape(x)[0]): # Percorre todas as possibilidades de entrada possíveis (4**hc)
        u1 = np.zeros(4) # Propulsor principal (tamanho do horizonte de predição)
        u2 = np.zeros(4) # Propulsor auxiliar (tamanho do horizonte de predição)
        for j in range(0, hc): # Note, que pela formulação, vamos até hc, pois o restante do vetor será zero, esse já definido
            if x[i, j]==1: # Motor de orientação para a esquerda
                u2[j] = 1 # Propulsor auxiliar recebe 1
            if x[i, j]==3: # Motor de orientação para a direita
                u2[j] = -1 # Propulsor auxiliar recebe -1
            if x[i, j]==2: # Acionamento do propulsor principal
                u1[j] = 1 # Propulsor principal recebe 1
        u1 = u1.reshape(-1, 1) # Transpõe uma matriz linha, logo u1 passa a ser uma matriz coluna
        u2 = u2.reshape(-1, 1) # Transpõe uma matriz linha, logo u2 passa a ser uma matriz coluna
        y_x = (fx.dot(u2)+lx)  # Predição de \hat{p}_x = resposta forçada + resosta livre
        y_y = (fy.dot(u1)+ly) # Predição de \hat{p}_y = resposta forçada + resosta livre
        y_t = (ft.dot(u2)+lt) # Predição de \hat{\theta} = resposta forçada + resosta livre
        y_v = (fv.dot(u1)+lv) # Predição de \hat{v}_y = resposta forçada + resosta livre
        w = np.array([1, 0, 1]) # Pessos iniciais iguais para o setpoint de angular e posicional (1) e nulo para velocidade - Estágio 1
        c = y_x 
        if abs(y_y[0])>abs(y_x[0]): # A ideia é acelerar a sonda, já que p_y \approx p_x, pega-se o que possui o maior termo inicial
            c = y_y
        ref = -p(np.abs(c)) # O polinômio referente o setpoint da velocidade recebe valores numéricos
        if y_v[0]<-ref[0]*.543:  # Esse if permite intercalar os dois estágios, se True, o estágio 1 atua 
            w = np.array([params[2], params[0], params[1]]) # Pesos calculados pelo algoritmo Subida de Encosta - Estágio 2
        if np.sum(pe)==2: # Verifica se ambos os pés estão em solo lunar
            w[2] = 0 # Caso afirmativo, o peso do setpoint angular é zero
        s[i] = sum(abs(w[0]*(y_x[:]-y_y[:])+w[1]*(y_v[:]+ref[:])-w[2]*(y_t[:]))) # Função Custo a ser minimizada
    action = x[np.argmin(s), 0] # Retorna com a primeira entrada do vetor de entra da que minimiza a Função Custo
    if sum(pe)==2 and py[k]<.1 and px[k]<.1: # Verifica se a sonda está dentro da plataforma com os pés fixados
        action = 0 # Caso True, não necessidade de mais ajuste, pois o obejetivo já foi alcançado
    return int(action) # Retorna a ação para aplicar no ambiente Lunar Lander

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


def run(params, env, verbose=False):
    """Executa um episódio dado parâmetros PID"""
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
    for trial in range(5):
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
    env._max_episode_steps = 450
   
    # RNGs de sementes
    np.random.seed(0)

    # Escalada aleatória da subida sobre params
    params = np.array([0, 0, 0])
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
