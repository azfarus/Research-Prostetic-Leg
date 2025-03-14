import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from simple_pid import PID
from scipy.linalg import solve_continuous_lyapunov

# Define system dynamics
m = 133
K = np.array([30,50]).reshape((2,1))
C = np.array([
    [-1,-.5,0,.5,1],
    [-1,-.5,0,.5,1]
])
Q = np.array([
    [500,0],
    [0,500]
])
Fai = np.array([
    [0,1],
    [-K[0,0],-K[1,0]]
])
A = Fai.T
P = solve_continuous_lyapunov(A,Q)
W = np.ones(5).reshape((5,1))*0.01
gamma = 1200
eta = 0.001
m_dash = 100
m_hat = 120
B = np.array([0,1]).reshape((2,1))
bj = 2


# tracking
position = []

def system_dynamics(t, state, u_func):
    x1, x2 = state
    u = u_func(state[0],state[1] , t)  # Control input function
    dx1 = x2
    dx2 = -25 * x2 - 10 * x1 + m * u  # Given dynamics
    return [dx1, dx2]

# Define control input (step input at t=2s)

def RBF(x):
    h = np.zeros(5).reshape((5,1))
    
    for i in range(5):
        h[i,0] = np.exp(-np.linalg.norm(x-C[:,i])**2/(2*bj**2))
        
    return h
def control_input(y,y_d, t):
    
    global W
    global m_hat , m_dash , gamma ,eta ,m
    yd =     np.sin(t)
    yd_d =   np.cos(t)
    yd_dd = -np.sin(t)
    
    e = yd-y
    e_d = yd_d-y_d
    
    E = np.array([e,e_d]).reshape((2,1))
    
    h = RBF(E)
    f = W.T @ h 
    
    etpb = E.T @ P @ B
    
    W_hat = -gamma * (etpb[0,0] * h)
    W -= W_hat
    
    u = (1.0/m_hat) * (-f + yd_dd + K.T @ E)
    
    etpbu = etpb[0,0] * u
    
    mdot = 0
    if  (etpbu > 0):
        mdot = (1/eta)*(etpbu)
    elif  (etpbu <= 0 and m_hat > m_dash):
        mdot = (1/eta)*(etpbu)
    else :
        mdot = 1/eta
    
    m_hat += mdot    
    
    position.append(yd)
    
    return u[0,0]  # Step input

# Initial conditions
x0 = [0.2, 0]  # Initial position and velocity

# Time span for simulation
t_span = (0, 100)  # Simulate for 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], 10000)  # Time points for evaluation

# Solve ODE
solution = solve_ivp(system_dynamics, t_span, x0, args=(control_input,), t_eval=t_eval)

# Plot results
plt.figure(figsize=(10, 5))
plt.plot(solution.t, solution.y[0], label="Position $x_1$")
# plt.plot(solution.t, solution.y[1], label="Velocity $x_2$")
# plt.plot(solution.t, np.sin(solution.t), label="Trajectory $x_2$")
plt.xlabel("Time [s]")
plt.ylabel("State Variables")
plt.title("Simulation of the Given System")
plt.legend()
plt.grid()
plt.show()
