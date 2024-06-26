# %% Read measured signals

# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# %% Read the data

path_prefix = '../measurements/2024-03-31/'
meas_file = 'signals_D2.log'
df = pd.read_csv(path_prefix + meas_file)

fig = plt.figure(meas_file)
plt.clf()

ax = fig.add_subplot(3,1,1)
ax.plot(1.0e-3*df['Time (ms)'][1:], df['Amplified Thermocouple voltage (V)'][1:],
         linestyle='-', marker='x')
ax.grid(True)
ax.set(ylim=(0.0, 3.3))
ax.set(xlabel='t (s)')
ax.set(ylabel='Voltage (V)')
ax.set(title='Amplified thermocouple voltage')

ax = fig.add_subplot(3,1,2)
ax.plot(1.0e-3*df['Time (ms)'][1:], df['Temperature (C)'][1:],
         linestyle='-', marker='x')
ax.grid(True)
ax.set(ylim=(10.0, 40.0))
ax.set(xlabel='t (s)')
ax.set(ylabel='Temperature (C)')
ax.set(title='Temperature')

ax = fig.add_subplot(3,1,3)
ax.plot(1.0e-3*df['Time (ms)'][1:], df['Pressure (Pa)'][1:],
         linestyle='-', marker='x')
ax.grid(True)
ax.set(ylim=(90000, 110000))
ax.set(xlabel='t (s)')
ax.set(ylabel='Pressure (Pa)')
ax.set(title='Pressure')

plt.show()

# %% Mittlere Spannung ausgeben

v_mean = df['Amplified Thermocouple voltage (V)'][1:].mean()
print(f'Mitlere Spannung Thermocouple Amplifier: {v_mean:7.3f} V')

