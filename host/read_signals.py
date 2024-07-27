# %% Read measured signals

# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# %% Read the data

path_prefix = './../measurements/2024-07-27/'
# Wenn man das Lab Supply nutzt und dann
# noch eine OpAmp Impedanzwandler Schaltung
# dann funktioniert das Signal
meas_file = 'signals_step_50_percent.log'
df = pd.read_csv(path_prefix + meas_file)

fig = plt.figure(meas_file, figsize=(7,12))
plt.clf()

ax = fig.add_subplot(3,1,1)
ax.plot(df['Time (ms)'][1:]/1000, df['Temperature (C)'][1:],
         linestyle='-')
ax.grid(True)
ax.set(ylim=(0, 300))
ax.set(xlabel='t (ms)')
ax.set(ylabel='Temperature (C)')
ax.set(title='Amplified thermocouple voltage')

plt.show()

# %% Offset removal and fitting of PT1

t = df['Time (ms)'] / 1000.0
theta = df['Temperature (C)']
# Do not start with 0 due to a non-existing meaningful measurement there.
theta_0 = np.mean(theta[1:40])
print(f'theta_0 = {theta_0} deg C')

def exp_fit(t, lag, amp, tau):
    y = amp*(1-np.exp(-(t-lag)/tau))
    y[t < lag] = 0.0
    return y

fig = plt.figure(meas_file, figsize=(7,12))
plt.clf()

ax = fig.add_subplot(3,1,1)
ax.plot(t, theta - theta_0, linestyle='-')
# Note that the step comes after 30 seconds, so the process lag is 30 seconds.
ax.plot(t, exp_fit(t, 30+30, 250, 320), linestyle='--')
# Tangente
tang = 250*(t-(30+30))/320
ax.plot(t, tang, linestyle='--')
ax.grid(True)
# ax.set(xlim=(-10,30))
# ax.set(ylim=(-10, 10))
ax.set(xlim=(-20,800))
ax.set(ylim=(-10, 250))
ax.set(xlabel='t (s)')
ax.set(ylabel='Temperature minus start temperature (C)')
ax.set(title='Amplified thermocouple voltage')

plt.show()


# %%
