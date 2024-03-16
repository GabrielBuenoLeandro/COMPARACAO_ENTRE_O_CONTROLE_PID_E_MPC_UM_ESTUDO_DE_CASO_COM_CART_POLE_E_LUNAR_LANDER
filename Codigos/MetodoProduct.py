from itertools import product
import numpy as np
import matplotlib.pyplot as plt

Hp = 4  # Opções de acordo com o delta u
p = np.array([-2, 0, 2])
b = product(p, repeat=Hp)  # Aplicação do método product

Possibilidades = []  # Lista vazia para receber todas as combinações
for i in b:
    Possibilidades.append(i)

# Matriz para organizar todas as possibilidades
Matriz = np.zeros((len(Possibilidades), Hp))
for i in range(0, len(Possibilidades)):
    Matriz[i, :] = np.array(Possibilidades[i])
