# Importando as Bibliotecas
import gym
from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import linregress


# Gráfico: "Resultados da aplicação de uma força constante à direita." 

env = gym.make("CartPole-v1", render_mode="human")
env.action_space.seed(42)
observation, info = env.reset(seed=42)
t = [] # Lista o episódio
P = [] # Lista para a Posição
V = [] # Lista para a Velocidade
A = [] # Lista para o ângulo
Va = [] # Lista para a Velocidade angular
for _ in range(100):
    observation, reward, terminated, truncated, info = env.step(1) # F esquerda
    P.append(observation[0]) # Guarda a Posição
    V.append(observation[1]) # Guarda a Velocidade
    A.append(observation[2]) # Guarda  o ângulo
    Va.append(observation[3]) # Guarda a Velocidade angular

    if terminated or truncated:
        observation, info = env.reset()
        t.append(_) # Guarda o término de cada episódio

env.close()
