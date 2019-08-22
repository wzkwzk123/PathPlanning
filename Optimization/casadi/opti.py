from casadi import *
import numpy as np

# need to define later
X_goal = 40
Y_goal = 2
V_ini = 30 # initial velocity condition
V_final = 0
#

T = 10.  # Time horizon
N = 50  # number of control intervals

# declare model variables
X = MX.sym("X")
Y = MX.sym("Y")
V = MX.sym("V")
Heading = MX.sym("Heading")
A = MX.sym("A") # acceration
Cur = MX.sym("Cur")

State = vertcat(X, Y, Heading, V, A, Cur) # [X Y FI V A K]

Jerk = MX.sym("Jerk")
Cur_rate = MX.sym("Cur_rate")

Control = vertcat(Jerk, Cur_rate)

# Model equations
State_dot = vertcat(V * cos(Heading), V * sin(Heading), V * Cur, A, Jerk, Cur_rate)

# objective term
L = (X - X_goal) ** 2 + (Y - Y_goal) ** 2

M = 4  # RK4 steps per interval
DT = T / N / M # 0.125
f = Function("f", [State, Control], [State_dot, L])
# State_ini = MX.zeros(6, 1)
# State_ini[3] = velocity
# State_goal = MX.zeros(6, 1)
# State_goal[0] = X_goal
# State_goal[1] = Y_goal

U = MX.sym("U",2)
X0  = MX.sym("X_0", 6)
X = X0
Q = 0

for j in range(M):
    k1, k1_q = f(X, U)
    k2, k2_q = f(X + DT/2, U)
    k3, k3_q = f(X + DT/2 * k2, U)
    k4, k4_q = f(X + DT/2 * k3, U)
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

# Evaluate at a test point
Fk = F(x0=[0.2,0.3, 0.4, 0.5, 0.6, 0.7],p= [0.2, 0.4])
print(Fk['xf'])
print(Fk['qf'])

# Start with an empty NLP
w=[]
w0 = []
lbw = []
ubw = []
J = 0
g=[]
lbg = []
ubg = [] #

# "Lift" initial conditions
Xk = MX.sym('X0', 6)
w += [Xk]
lbw += [0, 0, 0, V_ini, 0, 0] # initial condition
ubw += [0, 0, 0, V_ini, 0, 0]
w0 += [0, 0, 0, V_ini, 0, 0] # initial guess




# formulate the NLP
for k in range(N-1): # 20
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k), 2)
    w   += [Uk]
    lbw += [-1, 0] # the low bounder for accerlation and curvature -- control variables
    ubw += [3, 10] # upper bounder
    w0  += [-1, 0]

    # Integrate till the end of the interval
    Fk = F(x0=Xk, p=Uk)
    Xk_end = Fk['xf']
    J=J+Fk['qf']

    # New NLP variable for state at end of interval
    Xk = MX.sym('X_' + str(k+1), 6)
    w   += [Xk]
    lbw += [-100, -10, -inf, 0, -3, -inf] # [X Y FI V A K]
    ubw += [100, 10, inf, 10, 3, inf]
    w0 += [20, 0, 0, 20, -2, 0]

    # add equality constraint
    g += [Xk_end - Xk]
    lbg += [0, 0, 0, 0, 0, 0]
    ubg += [0, 0, 0, 0, 0, 0]

# the for final condition
Uk = MX.sym('U_' + str(N-1), 2)
w   += [Uk]
lbw += [-1, -5] # the low bounder for accerlation and curvature -- control variables
ubw += [3, 5] # upper bounder
w0  += [0, 0]

# Integrate till the end of the interval
Fk = F(x0=Xk, p=Uk)
Xk_end = Fk['xf']
J=J+Fk['qf']

# New NLP variable for state at end of interval
Xk = MX.sym('X_' + str(k+1), 6)
w   += [Xk]
lbw += [X_goal, Y_goal, -inf, V_final, -3, -inf] # [X Y FI V A K]
ubw += [X_goal, Y_goal, inf, V_final, 3, inf]
w0 += [0, 0, 0, 0, 0, 0]

# add equality constraint
g += [Xk_end - Xk]
lbg += [0, 0, 0, 0, 0, 0]
ubg += [0, 0, 0, 0, 0, 0]


# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob);
# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()

# Plot the solution
x1_opt = w_opt[0::8]
x2_opt = w_opt[1::8]
path = [x1_opt,x2_opt]
x3_opt = w_opt[3::8]
u_opt = w_opt[6::8]
print(x1_opt)

# u_opt = w_opt[2::8]
LL = len(x1_opt) # 21

tgrid = [T/N*k for k in range(LL)]

import matplotlib.pyplot as plt
plt.figure(1)
plt.clf()
plt.plot(tgrid, x1_opt, '--')
plt.plot(tgrid, x2_opt, '-')
plt.plot(tgrid, x3_opt, ':')
# plt.plot(tgrid, x3_opt, '-')


plt.step(tgrid, vertcat(DM.nan(1), u_opt), '-.')
plt.xlabel('t')
plt.legend(['x1','x2','u'])
plt.grid()

plt.figure(2)
plt.plot(x1_opt, x2_opt, "--")
plt.show()

