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

params =  np.array([58.52230249, 23.10399406,  3.69236577]) # Parâmetros calculados via algoritmo Subida de Encosta
num_episodios = 3 # Número de episódios
samples = int(2000*num_episodios) # Número de etapas (amostras)
px = np.zeros(samples) # Array para salvar a posição x (p_x)
py = np.zeros(samples) # Array para salvar a posição y (p_y)
p1 = np.zeros(samples) # Array para salvar a entrada u_1 (propulsor principal)
p2 = np.zeros(samples) # Array para salvar a entrada u_2 (propulsor auxiliar)
at = np.zeros(samples) # Array para salvar o ângulo (\theta)
v = np.zeros(samples) # Array para salvar a velocidade linear em y (v_y)
vx= np.zeros(samples) # Array para salvar a velocidade linear em x (v_x)
vt = np.zeros(samples) # Array para salvar a velocidade angular (v_{\theta})
r = [] # Lista para salvar a pontuação
#env = gym.make("LunarLander-v2", render_mode="human") # Criando o ambiente LunarLander e renderização, onde você pode alterar 'human' para 'rgb_array' ou 'ansi' dependendo do modo que deseja usar
env = gym.make("LunarLander-v2", render_mode="human", enable_wind=True, wind_power=3.5, gravity=-11.5, turbulence_power=1)
#env = gym.make("LunarLander-v2", render_mode="human", gravity=-1.63)
#env.action_space.seed() # Define a semente (seed) para a geração de números aleatórios no espaço de ações do ambiente
observation, info = env.reset(seed=47) # Reinicia o ambiente de simulação (LunarLander)
action = 0 # Seta a primeira ação
cont = 0 # Contador
g = [] # Lista que registra o término do episódio
inicio = time.time() # Inicia a contagem do tempo
for i in range(samples): # Percorre todo espaço amostral
    px[i] = observation[0]*(np.cos(np.pi/4))+observation[1]*(np.sin(np.pi/4)) # Salva posição x (p_x) rotacionada em 45°
    py[i] = observation[0]*(-np.sin(np.pi/4))+observation[1]*(np.cos(np.pi/4))  # Salva posição y (p_y) rotacionada em 45°
    at[i] = observation[4] # Salva o ângulo (\theta)
    v[i] = observation[3] # Salva a velocidade linear em y (v_y)
    vx[i] = observation[2] # Salva a velocidade linear em x (v_x)
    vt[i] = observation[5] # Salva a velocidade angular (v_{\theta})
    if i>=2: # Verifica se há atraso suficiente (2) para o sistema começão atuar
        action = MPC(px, py, i, at, v, params, pe) # Chama a função MPC com seus respectivos argumentos, que por sua vez retorna a ação de controle(action)
    observation, reward, terminated, truncated, info = env.step(action) # Aplicação da ação de controle
    pe = observation[6:] # Salva o posicionamento dos pés 
    r.append(reward) # Guarda a recompensa por etapa
    if reward==100: # Verifica se a sonda pousa
        cont+=1 # Caso afirmativo, é contabilizado
    if terminated or truncated: # Checa se o episódio terminou
        g.append(i) # Cada etapa de término é retida a lista g
        observation, info = env.reset(seed=47+len(g)) # Reinicia o ambiente de simulação (LunarLander)
        print("Episódio número: ", len(g)) # O tamanho de g indica os episódios completos
        if len(g)>=int(num_episodios): # Checa se os episódios atingiram o número determinado
            break # Encerra o ambiente
fim = time.time()
tempo_decorrido = fim - inicio # Calcula o tempo decorrido
print(f"Tempo decorrido: {tempo_decorrido} segundos")
env.close()

print("Número de pousos: ", cont)
d = 0
soma = np.zeros(3)
soma[0] = sum(r[:g[0]+1])
if soma[0]>=200:
    d = 1
# Plotando a pontuação de t
for i in range(0, 2):
    soma[i+1] = sum(r[g[i]:g[i+1]])
    if sum(r[g[i]:g[i+1]])>=200: # Verifica quais episódios possuem +200 pontos
        d+=1
plt.plot(np.arange(1, 4), soma, 'ko',  label='Episódio')
print("Número de pousos com +200 pontos: ", d)
print("Média de pontuação: ", np.mean(soma))
print("Número de etapas: ", len(r))
plt.plot(np.arange(1, 4), np.ones(3)*200, 'r',  linewidth=2.5, label='+200 pontos')
plt.ylabel('Pontuação')
plt.xlabel('Episódios')
plt.title('Validação - GPC')
plt.legend()
plt.show()
