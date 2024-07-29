# %% Read measured signals

# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctrl

# %% Read the data

path_prefix = './../measurements/2024-07-27/'
# Wenn man das Lab Supply nutzt und dann
# noch eine OpAmp Impedanzwandler Schaltung
# dann funktioniert das Signal
meas_file = 'signals_step_30_percent.log'
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

# Delay of the step function
step_delay = 30.0

pt1_delay_params = {
    'lag': 30.0,
    'amplitude': 250.0/50*30,
    'tau': 320.0
}

total_lag = step_delay + pt1_delay_params['lag']

def exp_fit(t, lag, amp, tau):
    y = amp*(1-np.exp(-(t-lag)/tau))
    y[t < lag] = 0.0
    return y

fig = plt.figure(meas_file, figsize=(7,12))
plt.clf()

ax = fig.add_subplot(3,1,1)
ax.plot(t, theta - theta_0, linestyle='-')
# Note that the step comes after 30 seconds, so the process lag is 30 seconds.
ax.plot(t, exp_fit(t, total_lag, pt1_delay_params['amplitude'], pt1_delay_params['tau']), linestyle='--')
# Tangente
tang = pt1_delay_params['amplitude']*(t-total_lag)/pt1_delay_params['tau']
ax.plot(t, tang, linestyle='--')
ax.grid(True)
# ax.set(xlim=(-10,30))
# ax.set(ylim=(-10, 10))
ax.set(xlim=(-20,800))
ax.set(ylim=(-10, 200))
ax.set(xlabel='t (s)')
ax.set(ylabel='Temperature minus start temperature (C)')
ax.set(title='Amplified thermocouple voltage')

plt.show()


# %% Ziegler-Nichols

# Amplitude considering the step height
# Step size of 50% = 0.5 --> Fitted amplitude = 250.0 deg C
# Step size of 30% = 0.3 --> Fitted amplitude = 150.0 deg C
# Extrapolated for step size of 1 --> Amplitude is 500 deg C
# Lag is 30 seconds
# Time constant tau is 320 seconds

# Slope R
R = 500.0 / pt1_delay_params['tau']
L = pt1_delay_params['lag']

# PI controller
kP = 0.9 / (R*L)
TI = L/0.3
kI = kP/TI

print(f'kP = {kP}')
print(f'TI = {TI}')
print(f'kI = {kI}')

# %% Simulation

tf1 = ctrl.tf([500.0], [pt1_delay_params['tau'], 1.0])
num, den = ctrl.pade(30, 5)
Td = ctrl.tf(num, den)
G = ctrl.series(Td, tf1)

t, h = ctrl.step_response(G)
plt.plot(t,h)
# plt.xlim(0,300)
# plt.ylim(-10,300)
plt.grid(True)
plt.show()

# %% Test the controller

s = ctrl.tf('s')
K = kP + kI/s

closed_loop = ctrl.feedback(ctrl.series(K, G))

t, h = ctrl.step_response(closed_loop)
plt.plot(t,h)
plt.grid(True)
plt.show()

K_disrete = ctrl.sample_system(K, 0.1)
print(K_disrete)

# %%
