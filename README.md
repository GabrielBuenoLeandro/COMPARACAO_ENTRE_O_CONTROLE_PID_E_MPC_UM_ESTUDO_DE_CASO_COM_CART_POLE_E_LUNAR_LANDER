# Controle PID e MPC Aplicado aos Ambientes CartPole e LunarLander

 Este repositório contém os códigos utilizados no meu Trabalho de Final de Curso (TFC). 

 # CartPole - Implementação do PID
 

  O primeiro passo é modelar o sistema de pêndulo invertido (CartPole), a ideia é encontrar como a entrada é dinamicamente transferida para a saída através da função de transferência do sistema, que para o sistema pêndulo invertido é a força aplicada na lateral ($u$) e a saída o ângulo do pêndulo ($\theta$):


<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/5b909e59-ac82-4594-8147-c86c43f08cd0" alt="Figura1">
</p>

 Para deduzir as equações de movimento do sistema, será considerado o diagrama de corpo livre do sistema de pêndulo invertido:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/b090f698-71ce-40e3-9e8a-f6f44d95741d" alt="Figura2">
</p>

o movimento horizontal do carro é dado pela equação:

$$
 M\frac{d^2x}{dt} = u - H.
$$

 A representação do deslocamento do centro de massa do pêndulo ($x_c$) é ilustrado na Figura abaixo:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/afe4a17a-8c2e-4b06-9a35-263bd3fc4660" alt="Figura3">
</p>

 Na Figura acima, observa-se que a coordenada $x_c$ do carrinho é obtida pela soma do deslocamento horizontal do centro de massa da coordenada que se encontra, representado como $x_c=x + \ell sin\theta$, e tomando a segunda derivada de $x_c$ e efetuando o balanço das forças horizontais, conforme a equação abaixo:

$$
m\frac{d^2}{dt}(x+\ell sen\theta) = H,
$$

o $y_c$ pode ser encontrado subtraindo $\ell sen\theta$ de $\ell$, agora basta tomar a sua segunda derivada e igual a resultante de forças verticais ($mg-V$):

$$
m\frac{d^2}{dt}(\ell-\ell cos\theta) = mg - V,
$$

no entanto, como $\ell$ é uma constante e sua segunda derivada é zero, a equação acima pode ser reformulada da seguinte maneira:

$$
m\frac{d^2}{dt}(-\ell cos\theta) = mg - V \Longrightarrow  m\frac{d^2}{dt}(\ell cos\theta) = V-mg.
$$

Próximo passo é encontrar o deslocamento rotacional:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/97aab695-b4b8-4642-b335-dacb25efa05d" alt="Figura4">
</p>

o  movimento rotacional da haste do pêndulo ao redor de seu centro de gravidade é descrito pela equação:

$$
I\frac{d^2\theta}{dt} = V\ell sen\theta - H\ell cos\theta,
$$

 onde I é o momento de inércia de inércia da haste em relação ao centro de gravidade.

As Equações do  movimento horizontal , balanço das forças horizontais e forças verticais  descrevem o comportamento dinâmico do pêndulo invertido, mas tais equações possuem funções trigonométricas, como seno e cosseno, logo para poder linearizá-las, será adotado um valor muito pequeno de $\theta$, pois a ideia é manter o sistema estável, o que permite dizer que: $cos\theta \approx 1$ e $sen\theta \approx \theta$, logo:

$$
I\frac{d^2\theta}{dt} = V\ell\theta - H\ell,
$$

$$
M\frac{d^2x}{dt} = u - H,
$$

$$
m\frac{d^2}{dt}(x+\ell\theta) = H,
$$

$$
 m\frac{d^2}{dt}(\ell) = V - mg\Longrightarrow  0 = V-mg.
$$

note que $H$ e $V$ são desconhecidos, mas a priori não se trata de um problema, pois pela equação penúltima é possível expressar $H$ e na última equação tem-se que $V=mg$, levando ao próximo passo, substituir esses valores na primeira equação, obtem:

$$
I\frac{d^2\theta}{dt} = mg\theta\ell-\frac{d^2}{dt} (x+\ell\theta)\ell,
$$

ou ainda:

$$
(I + m\ell^2)\frac{d^2\theta}{dt^2} + m\ell\frac{d^2x}{dt^2} - mg\ell\theta = 0,
$$

 substituindo a aquação $m\frac{d^2}{dt}(x+\ell\theta) = H$ na equação $M\frac{d^2x}{dt} = u - H$, tem-se:

 $$
 \frac{d^2x}{dt^2} = u - m\frac{d^2}{dt^2} (x + \ell\theta),
 $$

 reorganizando os termos:

 $$
  M\frac{d^2x}{dt^2} + m\frac{d^2}{dt^2} (x + \ell\theta) = u,
 $$

 agora será necessário obter a função de transferência do sistema, logo naturalmente, é necessário passar a Equação $(I + m\ell^2)\frac{d^2\theta}{dt^2} + m\ell\frac{d^2x}{dt^2} - mg\ell\theta = 0$ e $M\frac{d^2x}{dt^2} + m\frac{d^2}{dt^2} (x + \ell\theta) = u$ para o domínio de Laplace com condições iniciais nulas, sendo como segue:

 $$
 (I + m\ell^2)s^2\Theta(s) + m\ell s^2 X(s) - mg\ell\Theta(s) = 0,
 $$

$$
 (M+m)s^2 X(s) + m\ell s^2\Theta(s) = U(s),
$$

a ideia se volta em obter a relação da saída ($\Theta (s)$) em relação a entrada ($U(s)$), por isso o $X(s)$ deve ser eliminado, logo:

$$
((I + m\ell^2)s^2 - mg\ell)\Theta(s) + m\ell s^2 X(s) = 0,
$$

$$
X(s) = -\frac{\{(I+m\ell^2)s^2-mg\ell\}}{m\ell s^2} \Theta (s),
$$

substituindo a equação acima na equação $(M+m)s^2 X(s) + m\ell s^2\Theta(s) = U(s)$, chega-se:

$$
-(M+m)\cancel{s^2} \frac{\{(I+m\ell^2)s^2-mg\ell\}}{m\ell \cancel{s^2}} \Theta (s) + m\ell s^2\Theta(s) = U(s),
$$
$$
\frac{[m^2\ell^2 s^2 - (M+m)\{(I+m\ell^2)s^2-mg\ell\}]}{m\ell}\Theta (s) = U(s),
$$

por fim, tem-se a função de transferência:

$$
\frac{\Theta (s)}{U(s)} = \frac{m\ell}{(m^2\ell^2-(M+m)(I+m\ell^2))s^2+(M+m)mg\ell}.
$$

Para o projeto do controle PID será adotado um ambiente Cartpole do Gym, que se trata de um pêndulo invertido, como equacionado acima, onde as entradas possíveis são:

| Número | Ação                              |
|:------:|:---------------------------------:|
|   0    | Empurre o carrinho para à esquerda|
|   1    | Empurre o carrinho para à direita |

Sendo a saída:

| Número | Observação               | Mínimo                 | Máximo                 |
|:------:|:------------------------|:-----------------------|:-----------------------|
|   0    | Posição do carrinho     | -4,8                   | 4,8                     |
|   1    | Velocidade do carrinho  | -Inf                   | Inf                     |
|   2    | Ângulo do polo          | ~ -0,418 rad (-24°)    | ~ 0,418 rad (24°)      |
|   3    | Velocidade angular do polo | -Inf                | Inf                     |

O equacionamento já está pronto, sendo necessário agora saber quais são os valores de massa, comprimento e momento de inércia, bem, na verdade, não precisa saber o valor exato de cada uma dessas grandezas, pode-se aproximá-las com simulação e um pouco de física. Então será aplicado uma força à direita e traçar como o sistema se comporta.  Como o ambiente fornece os valores de velocidade e velocidade angular, é possível plotá-los para encontrar alguns valores para as variáveis supracitadas:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/7ff82374-44fe-4b29-8b5b-c13c78005d68" alt="Grafico1">
</p>

Não se pode realmente obter muitas informações úteis da posição ou ângulo, mas o fato de que a velocidade e a velocidade angular são retas são muito importantes. Isso sugere que a simulação da carreta é fisicamente precisa, pois uma força constante está produzindo uma aceleração constante tanto linearmente quanto angularmente.

Agora será trabalhado as acelerações lineares e angulares apenas encontrando a inclinação dos seus respectivos gráficos. Para tal, será empregado linregress do pacote Scipy, os resultados encontrados são:

$$
\ddot{x} = 0,19524\frac{m^2}{s},
$$

$$
 \ddot{\theta} =-0,29775\frac{rad^2}{s},
$$

um fato interessante, é que esses valores permanecem constantes independentemente do número de simulações, o que leva a crer que o ambiente CartPole possui um embasamento físico por trás.

Para continuar, considere a figura abaixo:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/22110556-bcef-44f4-a10b-4475bd02d127" alt="Figura5">
</p>

a força à direita deve resultar um torque no sentido anti-horário conforme a figura acima, que pela convenção é positivo, se considerar a entrada uma força de $1$ à direita, deve se observar a região que o torque é positivo, pois para massa e comprimento muito grande, o torque será negativo, pois a força não conseguirá restabelecer o sistema, para a tal análise será tomada a equação abaixo:

$$
 (I + m\ell^2)\frac{d^2\theta}{dt^2} + m\ell\frac{d^2x}{dt^2} - mg\ell\theta = 0,
$$

como o intuito é mantê-lo equilibrado, $\theta \approx 0$, abrindo a possibilidade de reescrever a equação como segue:

$$
(I + m\ell^2)\frac{d^2\theta}{dt^2}   = - m\ell\frac{d^2x}{dt^2},
$$

onde $\frac{d^2\theta}{dt^2} = -0,29775 \frac{rad^2}{s}$ e $\frac{d^2x}{dt^2}  = 0,19524 \frac{m^2}{s}$, o valor de $m$ adotado será  $0,5 kg$, logo:

$$
I = \frac{0,0976\ell - 0,1488\ell^2}{0,29775},
$$

 a equação acima expressa $I$ em função de $\ell$. Ao variar $\ell$ de $0$ a $1m$, obtém-se o gráfico:


<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/9a838f8a-87b5-47fb-a2d2-4ed8fae0986e" alt="Grafico2">
</p>

note que o valor escolhido de $\ell$ apresenta $I$ positivo.

Agora, será estimada a massa do carro ($M$) a partir da equação:

$$
(M + m)\frac{d^2 x}{dt^2} + m\ell \frac{d^2 \theta}{dt^2} = u,
$$

substituindo os valores conhecidos, como $m = 0,5kg$, $u = 1N$ e $\ell = 0,65m$, obtém-se:

$$
M = \frac{u - m\ell\ddot{\theta}-m\ddot{x}}{\ddot{x}} = \frac{1 + 0,5\cdot 0,65\cdot 0,29775 - 0,5\cdot 0,19524}{0,19524} = 5,11754 kg.
$$

Os resultados encontrados/fixados foram:

| $M [kg]$ | $m [kg]$ | $\ell [m]$ | $I[N\cdot m]$        |
|----------|----------|------------|----------------------|
| 5,11754  | 0,5      | 0,65       | $1,8537\times 10^{-3}$|

Ao substituí-los na Função de transferência, chega-se:

$$
\frac{\Theta (s)}{U(s)} = \frac{0,325}{-1,0914s^2 + 17,9101}.
$$

A função de transferência faz sentido a priori, pois uma força para a direita deve resultar em um ângulo $\theta$ negativo. No entanto, é necessário validar a função de transferência com o ambiente CartPole do Gymnasium. Portanto, será aplicada a mesma entrada tanto no ambiente CartPole quanto na função de transferência, considerando apenas um episódio (até o sistema perder a estabilidade). A cada episódio, o ambiente é reiniciado. Para adequar a entrada, será empregada a seguinte formulação: no CartPole, a entrada $1$ significa ir para a direita, enquanto $0$ é o comando para ir à esquerda. Para a função de transferência, o movimento para a direita também será representado por $1$, porém, para a esquerda, será representado por $-1$. Assim, tem-se:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/6560787a-0ec9-426c-848e-dc34d0665c73" alt="entval">
</p>

A partir da figura acima, nota-se que a F.T não conseguiu representar o sistema adequadamente. A F.T. responde de forma muito lenta aos estímulos de entrada. Portanto, é necessário ajustá-la para obter respostas mais abruptas. Ao observar os passos anteriores, a aceleração considerada levou em conta $8$ amostras, com um intervalo de $1s$ entre amostras, para realmente encontrar a inclinação. No entanto, devido à natureza computacional, o tempo é significativamente menor. Assim, as acelerações em $x$ e angular serão multiplicadas por um fator ($k$) até que a F.T. alcance um desempenho satisfatório. Dessa forma, será retornado a equação da inércia:

$$
(I + m\ell^2)\frac{d^2\theta}{dt^2}\cancel{k}   = - m\ell\frac{d^2x}{dt^2} \cancel{k},
$$

logo, o momento de inércia é independente do valor de $(k)$. Portanto, os valores de $(I)$ e $(\ell)$ permanecem inalterados, sendo $(1,8537\times 10^{-3} N\cdot m)$ e $(0,65 m)$, respectivamente.

Em teoria, à medida que a aceleração aumenta e a força permanece constante, a massa total deveria diminuir. Em termos matemáticos, isso é evidenciado pela Segunda Lei de Newton, onde $m$ é mantido constante em $0,5 m$, resultando em uma diminuição de $M$ (massa total), conforme descrito a seguir:

$$
 M = \frac{u - k m\ell\ddot{\theta}-k m\ddot{x}}{k\ddot{x}},
$$

um aspecto crucial a ser considerado é que a massa do carrinho deve ser um valor positivo ($M > 0$). Portanto:

$$
   0 < \frac{u - k m\ell\ddot{\theta}-k m\ddot{x}}{k\ddot{x}} \Rightarrow 0 < 1 + k\cdot 0,5\cdot 0,65\cdot 0,29775-k\cdot 0,5\cdot 0,19524,
$$

ao resolver a inequação acima, chega-se a $k<1174,74$, logo, para determinar o valor de ($k$), ele será variado de 1 a 1174, considerando 1000 etapas (amostras). Um conjunto de etapas compõe os episódios, sendo que cada episódio é concluído quando o ângulo do polo não está mais na faixa de $( \pm 12^\circ) (-0,2095 a 0,2095)$. Sendo como segue:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/61e0f5b0-afe8-43e4-8a04-7ea39f6c2444" alt="episodio">
</p>

 totalizando, são realizados 47 episódios completos. Portanto, para cada valor de $(k)$, a simulação é repetida 47 vezes, e o somatório do Erro Quadrático Médio $(RMSE)$ é calculado. Em seguida, é determinada a média para obter o $(RMSE)$ médio associado a $(k)$, da seguinte forma:


<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/c36d1c00-9272-4884-ad29-78a15c37927f" alt="eq">
</p>

onde $\hat{y}_k(i)$ representa a simulação livre para cada episódio (Função de Transferência) e ${y}_k$ é o sinal medido para cada episódio, com a média ($\bar{y}_k$) sendo calculada na janela de identificação. Graficamente:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/f1ed2ce7-21e3-4d0c-b34c-16de21b06cee" alt="RMSE1">
</p>

 ao utilizar o comando min do Python, obtém-se um valor de $\overline{RMSE}$ igual a $0,66019$ para $k=165$. Ao tomar $k=165$, obtém-se uma F.T. que responde melhor aos estímulos da entrada. A nova massa a ser:

$$
    M = \frac{1 + 165\cdot0,5\cdot0,65\cdot0,29775-165\cdot0,5\cdot0,19524}{165\cdot0,19524} = 0,02668kg,
$$

com os novos valores dos parâmetros, a Função de Transferência passa a ser:

$$
 \frac{\Theta (s)}{U(s)} = \frac{0,325}{-0,00661s^2 +  1,67919},
$$

 considerando a respostas ao degrau:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/9545890c-e1b7-403a-afad-e169c004f0a3" alt="step2">
</p>

Uma abordagem adicional envolve a identificação das raízes do polinômio no denominador da Função de Transferência, que são 16,3391 e -16,3391. É crucial notar que uma destas raízes está localizada no semiplano direito do eixo real, indicando a presença de um sistema instável.

Como exemplo do desempenho da FT, uma entrada será aplicada ao sistema para analisar como ele se comporta:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/9301adbb-06e6-417e-ab2c-6cc2b2fae07c" alt="entval2">
</p>

Logo, pode-se representar o diagrama de blocos em malha aberta do sistema:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/a55dcd4c-8a33-4ef4-b077-599bda8f5321" alt="DBFT">
</p>

Onde $U(s)$ representa a direção da força aplicada (com 1 indicando para a direita e -1 para a esquerda), enquanto $\Theta$ refere-se ao ângulo do pêndulo.

## Sintonia do PID

Para estabilizar o sistema, será introduzido um controlador PID (Proporcional, Integral e Derivativo). Um controlador PID  é composto por três elementos ajustáveis, os quais são adaptados com base na discrepância entre um ponto de ajuste definido $(r(t))$ e uma variável de processo medida $(y_m(t))$:

$$
 e(t) = r(t) - y_m(t).
$$

A saída de um controlador PID $(u(t))$ é determinada pela soma dos termos Proporcional, Integral e Derivativo, onde $K_P$, $K_I$ e $K_D$ são constantes ajustáveis que podem ser modificadas para otimizar o desempenho do controlador:

$$
 g_c(t) = K_P \cdot e(t) + K_I\cdot \int_0^t e(t) dt + K_D \cdot \frac{de(t)}{dt},
$$

passando para o domínio de Laplace:

$$
 G_c(s) = K_P \cdot e(t) + K_I\cdot \frac{1}{s}+ K_D \cdot s = \frac{K_D\cdot s^2 + K_P \cdot s + K_I}{s},
$$

Com o PID já introduzido, a equação do PID no domínio no tempo será reescrita da seguinte maneira:

$$
 g(t) = K_C \cdot e(t) + \frac{K_C}{\tau_I}\cdot \int_0^t e(t) dt + K_C\cdot \tau_I \cdot \frac{de(t)}{dt},
$$

observe que $(K_P = K_C)$, $(K_I = \frac{K_C}{\tau_I})$ e $(K_D = K_C \cdot \tau_D)$. Portanto, no domínio de Laplace, a expressão para um controlador PID pode ser representada da seguinte maneira:

$$
 G_c(s)  = \frac{K_C \cdot \tau_D\cdot s^2 + K_C \cdot s + \frac{K_C}{\tau_I}}{s}.
$$

Será implementada uma realimentação negativa no sistema representado na figura abaixo, uma vez que um sistema de malha fechada com PID é uma abordagem comum em controle de sistemas. Neste arranjo, o controlador ajusta dinamicamente a saída, respondendo à diferença entre a saída desejada e a saída real:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/c6d4666b-d6d4-47aa-8e88-3ce846358f55" alt="PIDneg">
</p>


Na determinação dos valores de $K_P$, $K_I$ e $K_D$, o pacote GEKKO será empregado. Este pacote, uma ferramenta em Python dedicada ao aprendizado de máquina e otimização de inteiros mistos, bem como a equações algébricas diferenciais, utilizará a equação do PID. Nessa equação, $\tau_i$ é igual a 2 e $\tau_d$ é igual a 0,25. Posteriormente, o pacote fornecerá o valor de $K_C$. A partir desse ponto, os parâmetros podem ser facilmente determinados. O ponto em questão será considerado como $\theta=0$ $(r(t))$. Dado que o processo envolve minimização, a metodologia a ser adotada é a seguinte:

$$
 e(t) = r(t) - y_m(t),
$$

o GEKKO, ao buscar a minimização do erro, simplifica o processo, necessitando apenas da introdução da Função de Transferência e da aplicação do impulso à entrada. Os valores estimados estão vinculados ao $K_C$ determinado, o qual é 0,5, os valores de $\tau_i$ e $\tau_d$ foram previamente estabelecidos como 2 e 0,25, respectivamente:

| Termos       | Relação com $K_C$ | Valor |
|--------------|-------------------|-------|
| Proporcional | $K_C$             | 0,5   |
| Integral     | $\frac{K_C}{\tau_I}$ | 0,25 |
| Derivativo   | $K_C \cdot \tau_D$  | 0,125 |

 os parâmetros $K_P$, $K_I$ e $K_D$ serão inseridos na equação do PID, levando a:

 $$
 g(t) = 0,5 \cdot e(t) + 0,25\cdot \int_0^t e(t) dt + 0,125 \cdot \frac{de(t)}{dt}.
 $$

 Ainda é necessário determinar o valor do erro, sua integral e derivada no contexto do controle de um pêndulo invertido (cartpole). Dado que o setpoint almejado (representado por $( r(t) )$ ) é manter o pêndulo em pé, idealmente, esse setpoint é zero. Assim, o erro torna-se o próprio ângulo do cartpole, enquanto a derivada é a velocidade angular, ambas informações fornecidas pela biblioteca Gymnasium.

 Além disso, é crucial estimar a integral do erro. Fisicamente, a integral pode ser interpretada como a soma acumulativa da posição ao longo do tempo. Este componente integral no controle é valioso para corrigir o erro acumulado ao longo do tempo, contribuindo para a estabilidade do sistema e a redução de desvios significativos. Matematicamente a integral é dada por:

 $$
 \int_0^t e(t) dt = \sum_{i=0}^n \theta_i,
 $$

aqui, $\theta_i$ representa o ângulo do pêndulo invertido, e $n$ é o número atual de episódio.

Portanto, ao considerar o ângulo, a velocidade angular e a integral do erro, é possível formar um conjunto abrangente de informações para implementa o PID no ambiente do pêndulo invertido, da seguinte maneira:

$$
 u_n = 0,5 \cdot \theta_n + 0,25\cdot \sum_{i=0}^n \theta_i + 0,125 \cdot \omega_n.
$$

antes de avançar com as tarefas, é pertinente realizar uma análise gráfica da saída do GEKKO, uma vez que essa visualização servirá como alicerce para as etapas subsequentes deste trabalho:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/d8752281-a41c-414c-af0d-0945d3ddab08" alt="pid">
</p>

observe que a saída $(u)$ assume valores contínuos, mas o ambiente cartpole utilizado é discreto (1 para força à direita e 0 para força à esquerda). Para resolver esse problema, será implementada a seguinte estratégia: se $(u > 0)$, será aplicada uma força à direita; caso contrário, será aplicada uma força à esquerda.

A etapa final consiste em executar a rotina e avaliar o desempenho do sistema do pêndulo invertido. Em resumo, isso proporcionará uma visão direta da sua estabilidade:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/d2f5f1ba-7bbc-4c34-86e6-9aebcadd50c5" alt="ctpidio">
</p>

O PID se mostrou capaz de controlar o CartPole, ou seja, missão concluída por enquanto:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/0be0e471-958a-4f2d-8f04-b087f9c8f03e" alt="cartpolepid">
</p>

# LunarLander - Implementação do MPC

Controle Preditivo Modelado (MPC) é uma estratégia que se baseia na previsão do comportamento futuro por meio de um modelo do sistema. Neste contexto, foi adotado a abordagem do Controle Preditivo Generalizado (GPC), que utiliza modelo paramétricos para prever o comportamento futuro do sistema. A obtenção dos modelos será via técnicas de identificação de sistemas, será usado o pacote SysIdentPy.

A trajetória de referência ($w$) representa o comportamento do sinal desejado para a saída futura, sendo o primeiro passo na aplicação do Controle Preditivo Generalizado ($GPC$). No caso do pêndulo invertido, ela será 0, pois a ideia é manter o pêndulo estável, o que implica em minimizar o $\theta$, de maneira:

$$
  w = [0, 0, 0, \cdots, n_{ep}],
$$

considerando que $n_{ep}$ representa o número de episódios.

O próximo passo consiste em estabelecer uma função de custo, considerando as recompensas do ambiente CartPole . Nesse contexto, uma recompensa é atribuída por cada passo dado, abrangendo inclusive a etapa de encerramento, uma vez que o objetivo é manter o poste ereto pelo maior tempo possível. O limite estabelecido para as recompensas é de 500. Logo, o esforço de controle não será penalizado, apenas o erro em relação à predição da saída e à referência:

$$
J(k) =\sqrt{(\sum_{j=d}^{h_p} [\hat{y}(j+k|k) - w(j+k)])^2},
$$

 a função de custo utilizada enfatiza a capacidade do parâmetro $\theta$ de mudar de sinal, possibilitando sua proximidade ao limiar de 0 radianos.

No entanto, ainda é necessário identificar um modelo capaz de prever a saída ($\hat{y}$) em função de $\Delta u$, seguindo a metodologia do GPC. Nesse contexto, a saída é representada por 1 para movimento à direita e -1 para movimento à esquerda, a fim de manter consistência com a codificação utilizada na Função de Transferência.  Isso resulta em $\Delta u$ assumindo três valores inteiros, o que se justifica por:

$$
    \begin{matrix}
        u_{k-1} = 1 \text{ e } u_{k} = -1 \Rightarrow \Delta u = -2 \\
        u_{k-1} = 1 \text{ e } u_{k} = 1 \Rightarrow \Delta u = 0\\
        u_{k-1} = -1 \text{ e } u_{k} = -1 \Rightarrow \Delta u = 0\\
        u_{k-1} = -1 \text{ e } u_{k} = 1 \Rightarrow \Delta u = 2 \\
    \end{matrix}.
$$

A definição do problema de otimização é a seguinte:

<p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/3547bbb5-40fa-4b90-ad5d-dcf804e4fb1c" alt="fcc""width="875">
</p>

a ferramenta adotada para minimizar essa função custo é o método de pesquisa em grade.

## Algoritmo Hill Climbing (subida da encosta)

O Método de Subida de Encosta é um algoritmo clássico para otimização, mostrando-se altamente eficaz na identificação de máximos ou mínimos locais. No processo desse algoritmo, inicia-se a partir de um ponto aleatório X e realiza-se sua avaliação. Em seguida, ocorre o deslocamento do ponto original X para um novo ponto vizinho, designado como X'. Se o novo ponto X' representar uma solução superior à do ponto anterior, permanece-se nele e o processo é repetido. Contudo, se for inferior, retorna-se ao ponto inicial X e tenta-se explorar outro vizinho. Uma das principais "restrições" do Método de Subida de Encosta é sua incapacidade de aceitar valores negativos; em outras palavras, ele sempre busca pontos vizinhos no espaço de solução que possam aprimorar seu estado atual. Caso não encontre tal aprimoramento, a execução é interrompida.

## Estimação do Modelo ARX

Agora será realizado o processo de identificação do modelo ARX do sistema, utilizando um sinal PRBS para excitar o CartPole. O PRBS assume apenas dois valores possíveis, $+V$ e $-V$. Além de ser fácil de implementar, é replicável, o que o torna bastante popular na identificação de sistemas. Como exemplo, considere o sinal $PRBS = [1, 1, 0, 1]$, representado graficamente:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/79111662-b3f3-4eae-bfef-21f4fff83387" alt="PRBS">
</p>

o menor intervalo no qual o nível do sinal é mantido é denominado $T_b$. Seu período pode ser determinado por $T = NT_b$, sendo $N$ um número ímpar, de modo que $T_b = T_s$ conforme representado na figura acima. Um resultado heurístico para a escolha de $T_b$ é:

$$
 \text{Lineares} \Leftarrow \frac{\tau_{min}}{10} \leq T_b \leq \frac{\tau_{min}}{3} \Rightarrow \text{Não lineares},
$$

 no trecho, onde $\tau_{min}$ representa a menor constante de tempo de interesse, sugere-se, de acordo com Nelles (2001), que, para sistemas lineares, $T_b$ seja escolhido próximo ao valor do intervalo de amostragem. Para sistemas não lineares, por outro lado, recomenda-se que $T_b$ seja aproximadamente igual a $\tau_{max}$, onde $\tau_{max}$ é a constante de tempo do sistema.

 Trabalhando com um modelo ARX, será considerado o tempo de amostragem $T_b$. Logo, são obtidos:

  <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/ca69ce64-4467-4c47-84c1-ba96d0070ab3" alt="teste">
</p>

é importante observar que o ambiente CartPole reinicia automaticamente sempre que atinge um desvio de aproximadamente $\pm 12^\circ$ (equivalente a $\pm 0,209$ radianos). Essa característica representa um desafio, pois resulta em episódios muito curtos, dificultando a estimativa do modelo ARX. Inicialmente, uma abordagem para solucionar esse problema poderia envolver a redução do intervalo de tempo $T_b$. Isso implicaria em uma diminuição no tempo de amostragem do sistema, permitindo que as ações sejam executadas em períodos mais curtos e, assim, estendendo a duração dos episódios.

No entanto, é crucial observar que essa abordagem não é viável no contexto do ambiente CartPole no Gymnasium. Isso se deve ao fato de que a taxa de amostragem no ambiente CartPole não é diretamente ajustável. O ambiente CartPole foi projetado para simular a física real associada ao problema de um pêndulo invertido sobre um carro, e a taxa de amostragem é intrínseca a essa simulação. Portanto, modificar diretamente a taxa de amostragem do ambiente não é uma opção disponível. Esse aspecto limita a capacidade de ajuste do tempo de amostragem para atender às necessidades específicas do modelo ARX no contexto do problema CartPole.

Para transpor esse desafio, o ambiente passará por dois milhões de iterações, e o episódio mais extenso (com o maior número de passos/amostras) será selecionado para a estimativa do modelo ARX. O episódio mais longo identificado consiste em 183 amostras:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/74ee5a2d-90d5-4d53-92ef-c3fe286ce211" alt="mep">
</p>

Será utilizado 158 amostras para o treinamento do modelo, sendo as 25 restantes empregadas para validação. O modelo ARX encontrado pelo SysIdentPy, foi:

$$
 y(k) = 2\cdot y(k-1)- 0,99367\cdot y(k-2)  - 0,00583\cdot u(k-1),
$$

sendo $RMSE = 0,0057$. Graficamente:

 <p align="center">
  <img src="https://github.com/GabrielBuenoLeandro/Controle_PID_MPC_CartPole_e_LunarLander/assets/89855274/7cda1a41-1128-4e3a-acab-8d5777a1358d" alt="mdm">
</p>

## Implementando o GPC

Seja o modelo ARX:

$$
\begin{split}
    y(k) + a_1 y(k-1) + a_2 y(k-2) + \cdots + a_n y(k-n) =\\
    =b_1 u(k-1) + b_2 u(k-2) + \cdots + b_n u(k-n),
\end{split}
$$

 para efetuar a predição um passo a frente, basta considerar $k=k+1$, tendo:

 $$
 \begin{split}
    y(k+1) + a_1 y(k) + a_2 y(k-1) + \cdots + a_n y(k-n+1) =\\
    =b_1 u(k) + b_2 u(k-1) + \cdots + b_n u(k-n+1),
\end{split}
 $$

em termos de valores preditos:

$$
\begin{split}
    \hat{y}(k+1|k) + a_1 y(k) + a_2 y(k-1) + \cdots + a_n y(k-n+1) =\\
    =b_1 \hat{u}(k|k) + b_2 u(k-1) + \cdots + b_n u(k-n+1),
\end{split}
$$

na forma matricial:

$$
\begin{split}
    \hat{y}(k+1|k) + 
\begin{bmatrix}
    a_1 & a_2 & \cdots & a_n \\
\end{bmatrix}
\begin{bmatrix}
    y(k)\\
    y(k-1)\\
    \vdots\\
    y(k-n+1)
\end{bmatrix}
=\\
=b_1 \hat{u}(k|k) + 
\begin{bmatrix}
    b_2 & b_3 & \cdots & b_n
\end{bmatrix}
\begin{bmatrix}
    u(k-1)\\
    u(k-2)\\
    \vdots\\
    u(k-n+1)
\end{bmatrix}
\end{split}
$$

agora considere a predição dois passos a frente ($k=k+2$):

$$
\begin{split}
    \hat{y}(k+2|k) + a_1 \hat{y}(k+1|k) + a_2 y(k) + \cdots + a_n y(k-n+2) =\\
    =b_1 \hat{u}(k+1|k) + b_2 \hat{u}(k|k) + \cdots + b_n u(k-n+2),
\end{split}
$$

em forma matricial:

$$
\begin{split}
    \begin{bmatrix}
        a_1 & 1
    \end{bmatrix}
    \begin{bmatrix}
        \hat{y}(k+1|k)\\
        \hat{y}(k+2|k)
    \end{bmatrix}
    +
    \begin{bmatrix}
        a_2 & a_3 & \cdots & 0\\
    \end{bmatrix}
\begin{bmatrix}
    y(k)\\
    y(k-1)\\
    \vdots\\
    y(k-n+1)
\end{bmatrix}
=\\
= \begin{bmatrix}
    b_2 & b_1\\
\end{bmatrix}
\begin{bmatrix}
    \hat{u}(k|k)\\
    \hat{u}(k+1|k)
\end{bmatrix}
+
\begin{bmatrix}
    b_3 & b_4 & \cdots & 0\\
\end{bmatrix}
\begin{bmatrix}
    u(k-1)\\
    u(k-2)\\
    \vdots\\
    u(k-n+1)\\
\end{bmatrix}
\end{split},
$$

predições até $N$ passos à frente ($N>n$):

$$
\begin{matrix}
&\hat{y}(k+1|k) + a_1 y(k) + a_2 y(k-1) + \cdots + a_n y(k-n+1) \\
&= b_1 \hat{u}(k|k) + b_2 u(k-1) + \cdots + b_n u(k-n+1)\\
&\hat{y}(k+2|k) + a_1 \hat{y}(k+1|k) + a_2 y(k) + \cdots + a_n y(k-n+2) \\
&= b_1 \hat{u}(k+1|k) + b_2 \hat{u}(k|k) + \cdots + b_n u(k-n+2)\\
&\hat{y}(k+3|k) + a_1 \hat{y}(k+2|k) + a_2 \hat{y}(k+1|k) + \cdots + a_n y(k-n+3) \\
&= b_1 \hat{u}(k+2|k) + b_2 \hat{u}(k+1|k) + \cdots + b_n u(k-n+3)\\
& \vdots\\
&\hat{y}(k+n|k) + a_1 \hat{y}(k+n-1|k) + \cdots + a_n y(k) \\
&= b_1 \hat{u}(k+n-1|k) + \cdots + b_n \hat{u}(k|k)\\
&\hat{y}(k+n+1|k) + a_1 \hat{y}(k+n|k) + \cdots + a_n \hat{y}(k+1|k) \\
&= b_1 \hat{u}(k+n|k) + \cdots + b_n \hat{u}(k+1|k)\\
& \vdots\\
&\hat{y}(k+N|k) + a_1 \hat{y}(k+N-1|k) + \cdots + a_n \hat{y}(k-n+N|k) \\
&= b_1 \hat{u}(k+N-1|k) + \cdots + b_n \hat{u}(k-n+N|k),
\end{matrix}
$$

 permitindo entender alguns padrões, o que possibilita uma implementação matricial para a equação de predição:

 $$
 \begin{matrix}
    \begin{bmatrix}
        1 & 0 & 0 & \cdots & 0 & 0 & \cdots & 0 & 0\\
        a_1 & 1 & 1 & \cdots & 0 & 0 & \cdots & 0 & 0\\
        a_3 & a_1 & 1 & \cdots & 0 & 0 & \cdots & 0 & 0\\
        \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots\\
        a_{n-1} & a_{n-2} & a_{n_3} & \cdots & 1 & 0 & \cdots 0 & 0\\
        a_n & a_{n-1} & a_{n-2} & \cdots & a_1 & 1 & \cdots & 0 & 0\\
        0 & a_n & a_{n-1} & \cdots & a_2 & a_1 & \cdots & 0 & 0\\
        \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots\\
        0 & 0 & 0 & \cdots & 0 & 0 & \cdots & 1 & 0\\
        0 & 0 & 0 & \cdots & 0 & 0 & \cdots & a_1 & 1\\
    \end{bmatrix}
    \begin{bmatrix}
        \hat{y}(k+1|k)\\
        \hat{y}(k+2|k)\\
        \hat{y}(k+3|k)\\
        \vdots\\
        \hat{y}(k+n|k)\\
        \hat{y}(k+n+1|k)\\
        \hat{y}(k+n+2|k)\\
        \vdots\\
        \hat{y}(k+N-1|k)\\
        \hat{y}(k+N|k)\\
    \end{bmatrix}+
   \end{matrix}
    $$
