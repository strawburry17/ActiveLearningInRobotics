# Finite-dimensional numerical trajectory optimization,
# moving a differential drive vehicle sideways

import numpy as np
from scipy.optimize import minimize
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Dynamical system (diff-drive car):
def dyn_sys(state, t, u):
    x, y, theta = state
    u1, u2 = u(t)
    dstate_dt = [np.cos(theta)*u1, np.sin(theta)*u1, u2]
    return dstate_dt

# Initial conditions:
x0 = [0, 0, np.pi/2]
print "x0 = ", x0

T = 2*np.pi # terminal time
N = 100 # number of timesteps
t = np.linspace(0,T,N) # array of timesteps
print t

# Initial control signal:
u0 = np.ones((2,N))
print "u0 = "
print u0
def u(time):
    timeindex, = np.where(t==time)
    return (u0[0][timeindex[0]],u0[1],[timeindex[0]])
print "u(t) = ", u(0)

Q = np.matrix([[1,0,0],[0,1,0],[0,0,1]]) # running cost weight on state error
P1 = np.matrix([[1,0,0],[0,1,0],[0,0,1]]) # temrinal cost weight on state error
R = np.matrix([[1,0],[0,1]]) # running cost weight on control effort

# objective function:
def obj_func(x_traj,u_traj):
    J = 0 # initialize value of objective function
    # Compute running cost:
    for i in range(1,N) :
        J += 1
    # Compute terminal cost:
    J += 1
    return J

print "Q = "
print Q
print "R = "
print R
print "P1 = "
print P1

sol = odeint(dyn_sys, x0, t, args=(u,))
print "Sol:"
print sol

# Plot state versus time:
fig1 = plt.figure(1)
plt.plot(t, sol[:, 0], 'b', label=r"$x(t)$")
plt.plot(t, sol[:, 1], 'g', label=r"$y(t)$")
plt.plot(t, sol[:, 2], 'r', label=r"$\theta(t)$")
plt.xlabel('t')
plt.legend(loc='best')
plt.grid()

# Plot y vs x:
fig2 = plt.figure(2)
plt.plot(sol[:, 0], sol[:,1], 'k', label=r"$(x(t), y(t))$")
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc='best')
plt.grid()


plt.show()
