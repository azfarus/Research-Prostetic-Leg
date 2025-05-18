import numpy as np
import matplotlib.pyplot as plt

def fourier5_ankle(t):
    
    x=t
    # Coefficients
    a0 = -0.007782
    a1 = -0.0204
    b1 = 0.0984
    a2 = 0.01139
    b2 = -0.1117
    a3 = -0.06086
    b3 = 0.01065
    a4 = 0.03639
    b4 = 0.002878
    a5 = 0.006763
    b5 = -0.02346
    w = 5.6

    # Fourier series computation
    f_x = (a0 + a1 * np.cos(x * w) + b1 * np.sin(x * w) +
           a2 * np.cos(2 * x * w) + b2 * np.sin(2 * x * w) +
           a3 * np.cos(3 * x * w) + b3 * np.sin(3 * x * w) +
           a4 * np.cos(4 * x * w) + b4 * np.sin(4 * x * w) +
           a5 * np.cos(5 * x * w) + b5 * np.sin(5 * x * w))

    # Derivatives
    df_x = (-a1 * w * np.sin(x * w) + b1 * w * np.cos(x * w) +
            -2 * a2 * w * np.sin(2 * x * w) + 2 * b2 * w * np.cos(2 * x * w) +
            -3 * a3 * w * np.sin(3 * x * w) + 3 * b3 * w * np.cos(3 * x * w) +
            -4 * a4 * w * np.sin(4 * x * w) + 4 * b4 * w * np.cos(4 * x * w) +
            -5 * a5 * w * np.sin(5 * x * w) + 5 * b5 * w * np.cos(5 * x * w))



    # Adjusted function
    f_x2 = f_x - a0
    df_x2 = df_x


    return f_x2, df_x2

def fourier5_knee(t):
    
    x=t
    # Coefficients
    a0 =      0.4246  #(0.4237, 0.4255)
    a1 =     -0.1041  #(-0.1059, -0.1023)
    b1 =     -0.3332  #(-0.3346, -0.3319)
    a2 =     -0.2508  #(-0.2525, -0.2491)
    b2 =      0.1945  #(0.1923, 0.1966)
    a3 =    0.003604  #(0.002036, 0.005172)
    b3 =     0.08631  #(0.08502, 0.0876)
    a4 =    -0.01391  #(-0.01522, -0.01261)
    b4 =     0.01127  #(0.009981, 0.01257)
    a5 =     0.00147  #(0.0001553, 0.002784)
    b5 =     0.01428  #(0.01298, 0.01557)
    w =       6.845   #(6.843, 6.847)

    # Fourier series computation
    f_x = (a0 + a1 * np.cos(x * w) + b1 * np.sin(x * w) +
           a2 * np.cos(2 * x * w) + b2 * np.sin(2 * x * w) +
           a3 * np.cos(3 * x * w) + b3 * np.sin(3 * x * w) +
           a4 * np.cos(4 * x * w) + b4 * np.sin(4 * x * w) +
           a5 * np.cos(5 * x * w) + b5 * np.sin(5 * x * w))

    # Derivatives
    df_x = (-a1 * w * np.sin(x * w) + b1 * w * np.cos(x * w) +
            -2 * a2 * w * np.sin(2 * x * w) + 2 * b2 * w * np.cos(2 * x * w) +
            -3 * a3 * w * np.sin(3 * x * w) + 3 * b3 * w * np.cos(3 * x * w) +
            -4 * a4 * w * np.sin(4 * x * w) + 4 * b4 * w * np.cos(4 * x * w) +
            -5 * a5 * w * np.sin(5 * x * w) + 5 * b5 * w * np.cos(5 * x * w))



    # Adjusted function
    f_x2 = f_x - a0
    df_x2 = df_x


    return f_x2, df_x2




def simple_sincos(t_inp):
    t = t_inp
    x = np.sin(t)
    dx = np.cos(t)
    
    return x, dx

# plt.plot(x, y, label='f(x)')
# plt.plot(x, dy, label="f'(x)")
# plt.plot(x, d2y, label="f''(x)")
# plt.title('Fourier5 Approximation for Ankle (Link 2)')
# plt.xlabel('x')
# plt.ylabel('Values')
# plt.grid(True)
# plt.legend()
# plt.show()