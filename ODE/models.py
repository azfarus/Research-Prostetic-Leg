import numpy as np
import matplotlib.pyplot as plt

def fourier5_ankle(x):
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

    d2f_x = (-a1 * w**2 * np.cos(x * w) - b1 * w**2 * np.sin(x * w) +
             -4 * a2 * w**2 * np.cos(2 * x * w) - 4 * b2 * w**2 * np.sin(2 * x * w) +
             -9 * a3 * w**2 * np.cos(3 * x * w) - 9 * b3 * w**2 * np.sin(3 * x * w) +
             -16 * a4 * w**2 * np.cos(4 * x * w) - 16 * b4 * w**2 * np.sin(4 * x * w) +
             -25 * a5 * w**2 * np.cos(5 * x * w) - 25 * b5 * w**2 * np.sin(5 * x * w))

    # Adjusted function
    f_x2 = f_x - a0
    df_x2 = df_x
    d2f_x2 = d2f_x

    return f_x2, df_x2, d2f_x2

# Generate data and plot
x = np.arange(0, 1, 0.001)
y, dy, d2y = fourier5_ankle(x)

# plt.plot(x, y, label='f(x)')
# plt.plot(x, dy, label="f'(x)")
# plt.plot(x, d2y, label="f''(x)")
# plt.title('Fourier5 Approximation for Ankle (Link 2)')
# plt.xlabel('x')
# plt.ylabel('Values')
# plt.grid(True)
# plt.legend()
# plt.show()