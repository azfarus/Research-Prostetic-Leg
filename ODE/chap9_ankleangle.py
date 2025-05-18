import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp,odeint
from custom_solvers import euler_solver,rk4_solver,adams_bashforth4_solver
from models import simple_sincos,fourier5_ankle


# Define system dynamics

b=10
L = 15
Fai = 15.0*np.eye(2)
Kv = 1.0*np.eye(2)
nodes = 7
c = np.vstack(
    (
        np.linspace(-15,15,nodes),
        np.linspace(-15,15,nodes),
    ),dtype=np.float64
)



def twoLinkSystem(q, q_dot, t , tau):
    p = np.array([2.9, 0.76, 0.87, 3.04, 0.87])
    (q1, q2) = q
    (q1_dot, q2_dot) = q_dot

    
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
        [p[3] *9.8* np.cos(q1) + p[4] *9.8* np.cos(q1 + q2)],
        [p[4] *9.8* np.cos(q1 + q2)]
    ])
    
    # Friction term F(q_dot)
    q_dot_vec = np.array([q1_dot, q2_dot]).reshape(2,1)
    F = 0.2 * np.sign(q_dot_vec)
    
    # Disturbance torque tau_d
    tau_d = np.array([0.1 * np.sin(t), 0.1 * np.sin(t)]).reshape(2,1)
    

    S =  np.linalg.inv(M) @ ( tau - V @ q_dot_vec - G - F - tau_d)
    
    return S

def system_dynamics( t,state, u_func , trajectory_func):

    q1 = state[0]
    q2 = state[2]
    dq1 = state[1]
    dq2 = state[3]
    
    w1 = state[4:4+nodes]
    w2 = state[4+nodes:4+2*nodes]
    
    u,q_dot,w_hat1,w_hat2 = u_func((q1,q2),(dq1,dq2) , t,w1,w2,trajectory_func)  # Control input function
    S = twoLinkSystem((q1,q2),(dq1,dq2),t,u)
    
    # print(S,u)
    dq1 = q_dot[0,0]
    ddq1 = S[0,0]  # Given dynamics
    dq2 = q_dot[1,0]  # Given dynamics
    ddq2 = S[1,0]  # Given dynamics
    
    dw1 = [0]*nodes
    dw2 = [0]*nodes
    
    for i in range(nodes):
        dw1[i] = w_hat1[i,0]
        dw2[i] = w_hat2[i,0]
    
    return np.array([dq1,  ddq1, dq2, ddq2]+dw1+dw2)



# Define control input (step input at t=2s)

def RBF(x):
    h = np.zeros(nodes).reshape((nodes,1))
    
    for i in range(nodes):
        norm = np.linalg.norm(x-c[:,i])
        h[i,0] = np.exp(-(norm**2)/(b*b))
        
    return h
def control_input(y,y_d,t,w1,w2,trajectory_func):
    
    W1=w1.reshape(nodes,1)
    W2=w2.reshape(nodes,1)

    
    q1d , q1d_d = trajectory_func(t)
    q2d , q2d_d = trajectory_func(t)

    
    e1 =   q1d-y[0]
    e1_d = q1d_d-y_d[0]
  
    e2 =   q2d-y[1]
    e2_d = q2d_d-y_d[1]
    
    e = np.array([e1,e2]).reshape(2,1)
    e_d = np.array([e1_d,e2_d]).reshape(2,1)
    
    r = e_d + Fai @ e
        
    z1 = np.array([e1,e1_d]).reshape(2,1)
    z2 = np.array([e2,e2_d]).reshape(2,1)
    
    
    h1 = RBF(z1)
    h2 = RBF(z2)
    
    W_hat1 = ( L/(1+np.linalg.norm(r)) ) * (h1 * r[0,0]) 
    W_hat2 = ( L/(1+np.linalg.norm(r)) ) * (h2 * r[1,0]) 
    

    fn = np.array([
        [(W1.T @ h1)[0,0]],
        [(W2.T @ h2)[0,0]]
    ])
    
    epN = 0.2  
    bd = 0.1
    v = -(epN+bd)*np.sign(r)
    
    tol = fn + Kv @ r - v
    
    q_dot = -r + np.array([q1d_d,q2d_d]).reshape(2,1) + Fai @ e
    
    return tol , q_dot  ,W_hat1,W_hat2

# Initial conditions
x0 = np.array([1,0.0,1, 0.0]+[0]*2*nodes)  # Initial position and velocity

# Time span for simulation
t_span = (0, 40)  # Simulate for 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], 100000)  # Time points for evaluation

# Solve ODE
# solution = solve_ivp(system_dynamics, t_span, x0, args=(control_input,), t_eval=t_eval)
trajectory_func = simple_sincos
solution = adams_bashforth4_solver(system_dynamics, t_span, x0, args=(control_input,trajectory_func), t_eval=t_eval)






fig,ax = plt.subplots(1,2,figsize=(10, 6))



ax[0].plot(solution['t'], solution['y'][0,:], label="position",c='r')
ax[0].plot(solution['t'], trajectory_func(solution['t'])[0], label="trajectory",c='g')
# ax[0].plot(solution['t'], trajectory_func(solution['t'])[0] - solution['y'][0,:] , label="error")

ax[1].plot(solution['t'], solution['y'][2,:], label="velocity",c='r')
ax[1].plot(solution['t'], trajectory_func(solution['t'])[0], label="trajectory",c='g')
# ax[1].plot(solution['t'], trajectory_func(solution['t'])[1] - solution['y'][1,:] , label="error")



plt.show()
