import numpy as np


def rk4_solver(system_dynamics, t_span, x0, args=(), t_eval=None):
    """
    Custom RK4 solver that follows the solve_ivp method signature.

    Parameters:
    - system_dynamics: function f(t, x, *args), returns dx/dt
    - t_span: tuple (t0, tf) specifying the time range
    - x0: initial state (array-like)
    - args: additional arguments for system_dynamics
    - t_eval: array of time points where solutions are computed

    Returns:
    - A dictionary containing:
        "t": time array
        "y": state array (each column corresponds to a time step)
    """
    t0, tf = t_span
    x0 = np.asarray(x0)
    
    # If t_eval is not provided, create an evenly spaced time array
    if t_eval is None:
        t_eval = np.linspace(t0, tf, 100)
    
    h = t_eval[1] - t_eval[0]  # Step size (assumes uniform spacing)
    num_steps = len(t_eval)
    num_states = len(x0)

    # Initialize storage for solutions
    y = np.zeros((num_states, num_steps))
    y[:, 0] = x0
    t = t_eval

    # RK4 Integration Loop
    x = x0
    for i in range(1, num_steps):
        ti = t[i-1]
        
        k1 = h * np.asarray(system_dynamics(ti, x, *args))
        k2 = h * np.asarray(system_dynamics(ti + h/2, x + k1/2, *args))
        k3 = h * np.asarray(system_dynamics(ti + h/2, x + k2/2, *args))
        k4 = h * np.asarray(system_dynamics(ti + h, x + k3, *args))
        
        x = x + (k1 + 2*k2 + 2*k3 + k4) / 6
        y[:, i] = x  # Store result
    
    return {"t": t, "y": y}

def euler_solver(system_dynamics, t_span, x0, args=(), t_eval=None):
    """
    Custom Euler method solver that follows the solve_ivp method signature.

    Parameters:
    - system_dynamics: function f(t, x, *args), returns dx/dt
    - t_span: tuple (t0, tf) specifying the time range
    - x0: initial state (array-like)
    - args: additional arguments for system_dynamics
    - t_eval: array of time points where solutions are computed

    Returns:
    - A dictionary containing:
        "t": time array
        "y": state array (each column corresponds to a time step)
    """
    t0, tf = t_span
    x0 = np.asarray(x0)
    
    # If t_eval is not provided, create an evenly spaced time array
    if t_eval is None:
        t_eval = np.linspace(t0, tf, 100)
    
    h = t_eval[1] - t_eval[0]  # Step size (assumes uniform spacing)
    num_steps = len(t_eval)
    num_states = len(x0)

    # Initialize storage for solutions
    y = np.zeros((num_states, num_steps))
    y[:, 0] = x0
    t = t_eval

    # Euler Integration Loop
    x = x0
    for i in range(1, num_steps):
        ti = t[i-1]
        dx = np.asarray(system_dynamics(ti, x, *args))
        x = x + h * dx  # Euler's method step
        y[:, i] = x  # Store result
    
    return {"t": t, "y": y}

def adams_bashforth4_solver(system_dynamics, t_span, x0, args=(), t_eval=None):
    """
    Custom Adams-Bashforth 4-step method solver assuming first three points are zero.

    Parameters:
    - system_dynamics: function f(t, x, *args), returns dx/dt
    - t_span: tuple (t0, tf) specifying the time range
    - x0: initial state (array-like)
    - args: additional arguments for system_dynamics
    - t_eval: array of time points where solutions are computed

    Returns:
    - A dictionary containing:
        "t": time array
        "y": state array (each column corresponds to a time step)
    """
    t0, tf = t_span
    x0 = np.asarray(x0)

    # If t_eval is not provided, create an evenly spaced time array
    if t_eval is None:
        t_eval = np.linspace(t0, tf, 100)

    h = t_eval[1] - t_eval[0]  # Step size (assumes uniform spacing)
    num_steps = len(t_eval)
    num_states = len(x0)

    # Initialize storage for solutions
    y = np.zeros((num_states, num_steps))  # Assume first three values are zero
    t = t_eval

    # Initialize function values for the first four steps
    f_values = np.zeros((4, num_states))  

    # Compute function values at the first three assumed zero points
    for i in range(3):
        f_values[i] = system_dynamics(t[i], np.zeros_like(x0), *args)

    # Apply AB4 method starting from the fourth step
    x = np.zeros_like(x0)  # Start from zero
    for i in range(3, num_steps - 1):
        ti = t[i]
        f_values[3] = system_dynamics(ti, x, *args)

        # AB4 formula: y[n+1] = y[n] + (h/24) * (55f[n] - 59f[n-1] + 37f[n-2] - 9f[n-3])
        x = x + (h / 24) * (55 * f_values[3] - 59 * f_values[2] + 37 * f_values[1] - 9 * f_values[0])
        
        # Store solution
        y[:, i+1] = x
        f_values[:-1] = f_values[1:]  # Shift stored f values

    return {"t": t, "y": y}