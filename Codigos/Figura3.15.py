from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  

m = GEKKO()
tf = 10
m.time = np.linspace(-10,tf,499)
step = np.zeros(499)
step[249] = 10


# Controller model
Kc = m.FV(value=1, lb=0.5, ub=1e6)  # controller gain
Kc.STATUS = 1
tauI = 2                 # controller reset time
tauD = 0.25              # derivative constant
OP_0 = m.Const(value=0.0)   # OP bias
OP = m.Var(value=0.0)       # controller output
PV = m.Var(value=0.04)       # process variable
SP = m.Param(value=step)    # set point
Intgl = m.Var(value=0.0)    # integral of the error
err = m.Intermediate(SP-PV) # set point error
m.Equation(Intgl.dt()==err) # integral of the error
m.Equation(OP == OP_0 + Kc*err + (Kc/tauI)*Intgl - PV.dt())
m.Obj(PV-SP)

# Process model
dydt = m.Var()
Kp = 1                    # process gain
tauP = 5                 # process time constant
m.Equation(dydt==PV.dt())
m.Equation(-0.006612887163570405*dydt.dt() + 1.679193272178997*PV == 0.325*OP)

m.options.IMODE=6
m.solve(disp=False)

plt.figure()
plt.subplot(2,1,1)
plt.plot(m.time,OP.value,'b:',label='u')
plt.ylabel('Saída do PID')
plt.legend()
plt.xlim(-0.01, 10)
plt.subplot(2,1,2)
plt.plot(m.time,SP.value,'k-',label='$y_m$')
plt.plot(m.time,PV.value,'r--',label='r')
plt.xlabel('Time (s)')
plt.ylabel('Processo')
plt.legend()
plt.xlim(-0.01, 10)
plt.show()

print('Valor de k_c', Kc.value[0])