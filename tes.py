import numpy as np
from scipy.signal import find_peaks, argrelextrema
import matplotlib.pyplot as plt

# Example signal
x = np.linspace(0, 10, 100)
y = np.sin(x) + 0.5 * np.random.randn(100)  # Signal with noise

# Find peaks
peaks, _ = find_peaks(y)

# Find local minima
minima = argrelextrema(y, np.less)[0]

extermums = np.hstack((peaks,minima))
# Number of peaks
num_peaks = len(peaks)
plt.plot(x,y)
for id in extermums:
    plt.scatter(x[id], y[id], color= 'red')
plt.show()

print(f'Number of peaks: {num_peaks}')
