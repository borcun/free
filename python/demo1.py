import numpy as np
import matplotlib.pyplot as plt

np.random.seed(19870718)

dt = 0.01
t = np.arange(0, 2, dt)
nse1 = np.random.randn(len(t))                 # white noise 1
nse2 = np.random.randn(len(t))                 # white noise 2

# Two signals with a coherent part at 10 Hz and a random part
s1 = np.sin(2 * np.pi * 10 * t) + nse1
s2 = np.sin(2 * np.pi * 10 * t) + nse2

plt.plot(t, s1, label='line1', linewidth=1)
plt.grid(True)
plt.show()
