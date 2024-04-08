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
#env = gym.make("LunarLander-v2", render_mode="human")
env = gym.make("LunarLander-v2", render_mode="human", enable_wind=True, wind_power=3.5, gravity=-11.5, turbulence_power=1)
#env = gym.make("LunarLander-v2", render_mode="human", gravity=-1.63)
p = []

def pid(state):
    """Agoritmo controle PID

    Args:
        state (ndarray): Estado da sonda lunar adivindo do espaço de observação do Lunar Lander

    Returns:
        int: Saída do controlador PID
    """
    # Parâmetros do PID
    kp_alt = 9.0565  # Proporcional da altitude
    kd_alt = -9.9488  # Deritivo da altitude
    kp_ang = 11.9271  # Proporcional do ângulo
    kd_ang = -5.0963  # Derivativo do ângulo
    
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

r = [] # Lista para salvar a recompensa
x = [] # Lista para salvar a posição x
y = [] # Lista para salvar a posição x
vx = [] # Lista para salvar a velocidade linear em x
vy = [] # Lista para salvar a velocidade linear em y
th = [] # Lista para salvar o ângulo
vt = [] # Lista para salvar a velocidade angular
g = [] # Salva o fim do episódio
cont = 0 # Início do contador do episódio
ooo = 0 # Início do contador de pouso
inicio = time.time() # Início da contagem de tempo
num_ep = 3 # Define o número de episódios
for t in range(3): # Vai até o episódio definido
    observation = env.reset(seed=47+t) # Criando o ambiente LunarLander e renderização, onde você pode alterar 'human' para 'rgb_array' ou 'ansi' dependendo do modo que deseja usar
    observation = observation[0] # Salva o espaço de observação
    while 1: # Verifica a continuidade, sempre continuando só para devido ao break da linha 91
        env.render() # Renderização
        cont+=1 # Contador recebe mais 1
        action = pid(observation) # Chama a função PID
        observation, reward, done, info, a = env.step(action) # Aplicação da ação de controle
        r.append(reward) # Salva a recompensa
        if reward==100: # Se pousar (reward==100)
            ooo+=1 # Contador de pousos
        x.append(observation[0]) # Salva a posição x
        y.append(observation[1]) # Salva a posição y
        vx.append(observation[2]) # Salva a velocidade linear em x
        vy.append(observation[3]) # Salva a velocidade linear em y
        th.append(observation[4]) # Salva o ângulo
        vt.append(observation[5]) # Salva a velocidade angular
        if done: # Verifica o término do episódio
            print("Episódio número: ", t+1)  # Printa a cada fim de episódio
            g.append(cont) # Salva o fim do episódio
            break # Para o algoritmo

env.close() # Encerra o ambiente
fim = time.time() # Marca o fim da contagem do tempo
tempo_decorrido = fim - inicio # Calcula o tempo decorrido
print(f"Tempo decorrido: {tempo_decorrido} segundos") # Printa o tempo decorrido
print("Número de pousos: ", ooo) # Printa o número de pousos com sucesso
d = 0 # Conta os pesos com +200 pontos
soma = np.zeros(num_ep) # Salva a soma para cada episódio
soma[0] = sum(r[:g[0]]) # Salva a soma do episódio inicial
if soma[0]>=200: # Verifica se a soma é mais de 200 pontos para ep zero
    d = 1 # Inicia o contador
for i in range(0, num_ep-1): # Percorre os episódios realizados
    soma[i+1] = sum(r[g[i]:g[i+1]]) # Soma para cada episódio
    if sum(r[g[i]:g[i+1]])>200: # Verifica se a soma é mais de 200 pontos para os eps restante
        d+=1 # O contador soma mais um a d
# Mostrando os resultados
plt.plot(np.arange(1, 4), soma, 'ko',  label='Episódio')
print("Número de pousos com +200 pontos: ", d)
print("Média de pontuação: ", np.mean(soma))
print("Número de etapas: ", len(x))
plt.plot(np.arange(1, 4), np.ones(3)*200, 'r',  linewidth=2.5, label='+200 pontos')
plt.ylabel('Pontuação')
plt.xlabel('Episódios')
plt.title('Validação - PD')
plt.legend()
plt.show()
