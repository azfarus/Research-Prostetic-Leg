import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp,odeint
from custom_solvers import euler_solver,rk4_solver,adams_bashforth4_solver
from models import fourier5_ankle,simple_sincos ,fourier5_knee


# Define system dynamics

b=10
Ll = 22 # weight gain
lamb = 6.0*np.eye(2)
Kv= 4.0*np.eye(2)
nodes = 10
c = np.vstack(
    (
    np.linspace(-2,2,nodes),
    np.linspace(-2,2,nodes),
    ),dtype=np.float64
)
mk = 2.63     # kg
ma = 0.82     # kg
lk = 0.19     # m
la = 0.06     # m
g = 9.81      # m/s^2
# trajectory = fourier5_ankle


# Assume q and q_dot are defined as numpy arrays: [q1, q2] and [q1_dot, q2_dot]
def compute_dynamics(q, q_dot):
    q1, q2 = q
    q1_dot, q2_dot = q_dot

    # Mass matrix M(q)
    M11 = (mk + ma) * lk**2 + ma * la**2 + 2 * ma * lk * la * np.cos(q2)
    M12 = ma * la**2 + ma * lk * la * np.cos(q2)
    M22 = ma * la**2
    M = np.array([[M11, M12],
                  [M12, M22]])

    # Coriolis and centrifugal terms V(q, q_dot)
    V1 = -ma * lk * la * (2 * q1_dot * q2_dot + q2_dot**2) * np.sin(q2)
    V2 = ma * lk * la * q1_dot**2 * np.sin(q2)
    V = np.array([V1, V2]).reshape(2,1)

    # Gravity vector G(q)
    G1 = (mk + ma) * g * lk * np.cos(q1) + ma * g * la * np.cos(q1 + q2)
    G2 = ma * g * la * np.cos(q1 + q2)
    G = np.array([G1, G2]).reshape(2,1)

    return M, V, G


def twoLinkSystem(q, q_dot, t , tau ):
    p = np.array([2.9, 0.76, 0.87, 3.04, 0.87])

    (q1_dot, q2_dot) = q_dot

    
    
    M,V,G = compute_dynamics(q, q_dot)
    
    # Friction term F(q_dot)
    q_dot_vec = np.array([q1_dot, q2_dot]).reshape(2,1)
    F = 0.2 * np.sign(q_dot_vec)
    
    # Disturbance torque tau_d
    tau_d = np.array([0.1 * np.sin(t), 0.1 * np.sin(t)]).reshape(2,1)
    

    # S =  np.linalg.inv(M) @ ( tau - V @ q_dot_vec - G - F - tau_d)
    S =  np.linalg.inv(M) @ ( tau - V  - G - F - tau_d)
    
    # print(q1_dot,q2_dot,q1,q2,t)
    return S

def system_dynamics( t,state, u_func):

    q1 = state[0]
    q2 = state[2]
    dq1 = state[1]
    dq2 = state[3]
    
    w1 = state[4:4+nodes]
    w2 = state[4+nodes:4+2*nodes]
    
    u,q_dot,w_hat1,w_hat2 = u_func((q1,q2),(dq1,dq2) , t,w1,w2)  # Control input function
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
def control_input(y,y_d,t,w1,w2):
    
    global trajectory
    W1=w1.reshape(nodes,1)
    W2=w2.reshape(nodes,1)

    # q1d =     0.1*np.sin(t)
    # q1d_d =   0.1*np.cos(t)

    
    # q2d =     0.1*np.sin(t)
    # q2d_d =   0.1*np.cos(t)
    
    q1d , q1d_d = fourier5_knee(t)
    q2d , q2d_d = fourier5_ankle(t)

    
    e1 =   q1d-y[0]
    e1_d = q1d_d-y_d[0]
  
    e2 =   q2d-y[1]
    e2_d = q2d_d-y_d[1]
    
    e = np.array([e1,e2]).reshape(2,1)
    e_d = np.array([e1_d,e2_d]).reshape(2,1)
    
    r = e_d + lamb @ e
        
    z1 = np.array([e1,e1_d]).reshape(2,1)
    z2 = np.array([e2,e2_d]).reshape(2,1)
    
    
    h1 = RBF(z1)
    h2 = RBF(z2)
    
    W_hat1 = Ll * (h1 * r[0,0]) 
    W_hat2 = Ll * (h2 * r[1,0]) 
    

    fn = np.array([
        [(W1.T @ h1)[0,0]],
        [(W2.T @ h2)[0,0]]
    ])
    
    epN = 0.2  
    bd = 0.1
    v = -(epN+bd)*np.sign(r)
    
    tol = fn + Kv @ r - v
    
    q_dot = -r + np.array([q1d_d,q2d_d]).reshape(2,1) + lamb @ e
    
    return tol , q_dot  ,W_hat1,W_hat2

# Initial conditions
x0 = np.array([0.0,0.0,0.0, 0.0]+[0]*2*nodes)  # Initial position and velocity

# Time span for simulation
t_span = (0, 5)  # Simulate for 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], 10000)  # Time points for evaluation

# Solve ODE
# solution = solve_ivp(system_dynamics, t_span, x0, args=(control_input,), t_eval=t_eval)
solution = rk4_solver(system_dynamics, t_span, x0, args=(control_input,), t_eval=t_eval)






fig, ax = plt.subplots(2, 1, figsize=(10, 6))

# --- First subplot: Knee position ---
ax[0].plot(solution['t'], solution['y'][0, :], label="Knee Position")
ax[0].plot(solution['t'], fourier5_knee(solution['t'])[0], label="Knee Trajectory")
ax[0].plot(solution['t'], fourier5_knee(solution['t'])[0] - solution['y'][0, :], label="Position Error")

ax[0].set_title("Knee Joint Position Tracking")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Angle [rad]")
ax[0].legend()
ax[0].grid(True)

# --- Second subplot: Ankle velocity ---
ax[1].plot(solution['t'], solution['y'][2, :], label="Ankle Velocity")
ax[1].plot(solution['t'], fourier5_ankle(solution['t'])[0], label="Ankle Trajectory")
ax[1].plot(solution['t'], fourier5_ankle(solution['t'])[0] - solution['y'][2, :], label="Velocity Error")

ax[1].set_title("Ankle Joint Velocity Tracking")
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Angular Velocity [rad/s]")
ax[1].legend()
ax[1].grid(True)

plt.tight_layout()
plt.show()
