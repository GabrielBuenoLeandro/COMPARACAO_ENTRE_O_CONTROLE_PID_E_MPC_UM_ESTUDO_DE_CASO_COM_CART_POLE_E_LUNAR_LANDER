# Controle PID MPC Para os Ambientes CartPole e LunarLander

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


