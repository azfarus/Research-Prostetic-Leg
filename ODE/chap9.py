import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# Define system dynamics
c = np.vstack(
    (
    np.array([-1.5,-1,-.5,0,.5,1,1.5]),
    np.array([-1.5,-1,-.5,0,.5,1,1.5])
    ),dtype=np.float64
)
print(c.shape)
b=10
L = 15*np.eye(2)
A = 5.0*np.eye(2)
Kv= 10.0*np.eye(2)

W1 = np.zeros(shape=(7,1))+0.0
W2 = np.zeros(shape=(7,1))+0.0



# tracking
position = []
trajectory = []

def twoLinkSystem(q, q_dot, t , tau):
    p = np.array([2.9, 0.76, 0.87, 3.04, 0.87])
    q1, q2 = q
    q1_dot, q2_dot = q_dot

    
    # Mass matrix M(q)
    M = np.array([
        [p[0] + p[1] + 2 * p[2] * np.cos(q2), p[1] + p[2] * np.cos(q2)],
        [p[1] + p[2] * np.cos(q2), p[1]]
    ])
    
    # Coriolis/centrifugal matrix V(q, q_dot)
    V = np.array([
        [-p[2] * q2_dot * np.sin(q2), -p[2] * (q1_dot + q2_dot) * np.sin(q2)],
        [p[2] * q1_dot * np.sin(q2), 0]
    ])
    
    # Gravity vector G(q)
    G = np.array([
        [p[3] * np.cos(q1) + p[4] * np.cos(q1 + q2)],
        [p[4] * np.cos(q1 + q2)]
    ])
    
    # Friction term F(q_dot)
    F = 0.02 * np.sign(np.array([q1_dot, q2_dot])).reshape(2,1)
    
    # Disturbance torque tau_d
    tau_d = np.array([0.2 * np.sin(t), 0.2 * np.sin(t)]).reshape(2,1)
    
    # Compute tau
    # tau = M @ np.array([q1_ddot, q2_ddot]) + V @ np.array([q1_dot, q2_dot]) + G + F + tau_d
    S =  np.linalg.inv(M) @ ( tau - V @ np.array([q1_dot, q2_dot]).reshape(2,1) - G - F - tau_d)
    
    return S

def system_dynamics(t, state, u_func):
    q1 , q1_d, q2 , q2_d = state
    u,q_dot = u_func((q1,q2),(q1_d,q2_d) , t)  # Control input function
    S = twoLinkSystem((q1,q2),(q1_d,q2_d),t,u)
    
    # print(S,u)
    dx1 = q_dot[0,0]
    dx2 = S[0,0]  # Given dynamics
    dx3 = q_dot[1,0]  # Given dynamics
    dx4 = S[1,0]  # Given dynamics

    return (dx1, dx2 , dx3 , dx4)

# Define control input (step input at t=2s)

def RBF(x):
    h = np.zeros(7).reshape((7,1))
    
    for i in range(7):
        h[i,0] = np.exp(-np.linalg.norm(x-c[:,i])**2/(b*b))
        
    return h
def control_input(y,y_d, t):
    
    global W1,W2
    global m_hat , m_dash , gamma ,eta ,m
    q1d =     0.1*np.sin(t)
    q1d_d =   0.1*np.cos(t)

    
    q2d =     0.1*np.sin(t)
    q2d_d =   0.1*np.cos(t)

    
    e1 =   q1d-y[0]
    e1_d = q1d_d-y_d[0]
  
    e2 =   q2d-y[1]
    e2_d = q2d_d-y_d[1]
    
    e = np.array([e1,e2]).reshape(2,1)
    e_d = np.array([e1_d,e2_d]).reshape(2,1)
    
    r = e_d + A @ e
    

    
    z1 = np.array([e1,e1_d]).reshape(2,1)
    z2 = np.array([e2,e2_d]).reshape(2,1)
    
    
    h1 = RBF(z1)
    h2 = RBF(z2)
    

    W_hat1 = 15 * (h1 * r[0,0]) 
    W_hat2 = 15 * (h2 * r[1,0]) 
    
    # print(W_hat1,W_hat1.shape)
    fn = np.array([
        [(W1.T @ h1)[0,0]],
        [(W2.T @ h2)[0,0]]
    ])
    
    epN = 0.2  
    bd = 0.1
    v = -(epN+bd)*np.sign(r)
    

    tol = fn + Kv @ r - v
    
    W1-=W_hat1
    W2-=W_hat2
    
    trajectory.append(q1d)
    position.append(y[0])
    
    q_dot = -r + np.array([q1d_d,q2d_d]).reshape(2,1) + A @ e
    
    return tol , q_dot  

# Initial conditions
x0 = (0.09,0.0,-0.09, 0.0)  # Initial position and velocity

# Time span for simulation
t_span = (0, 100)  # Simulate for 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], 100000)  # Time points for evaluation

# Solve ODE
solution = solve_ivp(system_dynamics, t_span, x0, args=(control_input,), t_eval=t_eval)

# Plot results
plt.figure(figsize=(10, 5))
# plt.plot(range(len(position)), position, label="Position $x_1$")
plt.plot(solution.t, solution.y[0], label="Velocity $x_0$")
plt.plot(solution.t, 0.1*np.sin(solution.t), label="Trajectory $x_2$")
plt.xlabel("Time [s]")
plt.ylabel("State Variables")
plt.title("Simulation of the Given System")
plt.legend()
plt.grid()
plt.show()
