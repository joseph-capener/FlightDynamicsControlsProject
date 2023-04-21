import sys

sys.path.append('..')
import numpy as np
import matplotlib.pyplot as plt

for i in range(0,20):
    print(np.random.normal(300,5))

# Speed of light in meters per second
c = 299792458

# Transmit frequency in Hertz
f_tx = 415e6

# Transmit power in watts
p_tx = 2e4

# Target distance in meters
d_target = 5e4

# Target velocity in meters per second
v_target = 100

# Sampling rate in Hertz
fs = 10e2

# Sampling period in seconds
T = 1 / fs

# Simulation duration in seconds
t_end = 1

# Number of samples
n_samples = int(t_end * fs)

# Time vector
t = np.arange(n_samples) * T

# Transmit signal
s_tx = np.sqrt(2 * p_tx) * np.cos(2 * np.pi * f_tx * t)

# Receive signal (with delay and Doppler shift)
s_rx = np.sqrt(2 * p_tx) * np.cos(2 * np.pi * f_tx * (t - 2 * d_target / c)) * np.exp(1j * 4 * np.pi * f_tx * v_target / c * d_target / c)

# Matched filter (to improve signal-to-noise ratio)
s_mf = np.flipud(s_tx) * np.conj(s_rx)

# Range-Doppler map (2D Fourier transform of matched filter output)
rd_map = np.fft.fft2(s_mf.reshape(-1, 1), (n_samples, n_samples))

# Range and Doppler axes
r_axis = c / (2 * fs) * np.arange(n_samples)
v_axis = c / (2 * f_tx * d_target) * np.fft.fftfreq(n_samples, T)

# Plot the range-Doppler map
plt.figure()
plt.imshow(np.abs(rd_map), aspect='auto', extent=[v_axis[0], v_axis[-1], r_axis[-1], r_axis[0]])
plt.xlabel('Doppler velocity (m/s)')
plt.ylabel('Range (m)')
plt.title('Range-Doppler Map')
plt.colorbar()
plt.show()