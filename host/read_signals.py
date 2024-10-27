# %% Read measured signals

# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctrl

# %% Define the signals to be read

# 100% PWM signal with stop after 100 deg C (October)
path_prefix = './../measurements/2024-10-27/'
meas_file = 'signals_step_100_percent_until_100_degC_cooling_down_door_closed.log'
signal_100_deg_C_stop = path_prefix + meas_file

# 100% PWM signal with stop after 200 deg C (October)
path_prefix = './../measurements/2024-10-27/'
meas_file = 'signals_step_100_percent_until_200_degC_cooling_down_door_closed.log'
signal_200_deg_C_stop = path_prefix + meas_file


# %% Read and plot the signals

df = pd.read_csv(signal_200_deg_C_stop, skiprows=2)

fig = plt.figure(meas_file, figsize=(10,5))
plt.clf()

ax = fig.add_subplot(1,1,1)
ax.plot(df['Time (ms)']/1000, df['Oven temperature (C)'],
         linestyle='-', label='Oven temperature (C)')
ax.plot(df['Time (ms)']/1000, df['Ambient temperature (C)'],
         linestyle='--', label='Ambient temperature (C)')
ax.plot(df['Time (ms)']/1000, df['PWM controller (percent)'],
         linestyle='--', label='PWM controller (percent)')
ax.plot(df['Time (ms)']/1000, df['Thermocouple open']*25,
         linestyle='--', label='Thermocouple open (x25)')
ax.grid(True)
ax.set(ylim=(0, 300))
ax.set(xlabel='t (ms)')
ax.set(title='Measured signals')
ax.legend()

plt.show()



# %% Simulation via plant model using the actual input signal

# Model fitted to the 100% PWM step that turns off once 100 deg C are reached.
tf_pt1_a = ctrl.tf([6.7], [30.0, 1.0])
tf_pt1_b = ctrl.tf([1.0], [480.0, 1.0])
num, den = ctrl.pade(22, 5)
tf_Td = ctrl.tf(num, den)
G = ctrl.series(tf_Td, tf_pt1_a, tf_pt1_b)


# t = df['Time (ms)'] / 1000.0
# t = t.to_numpy()
# Make sure t is equally spaced
t = np.arange(0,2*720+1)*0.5

theta = df['Oven temperature (C)'].to_numpy()
theta_0 = np.mean(theta[:40])
print(f'theta_0 = {theta_0} deg C')

u = df['PWM controller (percent)'].to_numpy()

time_response = ctrl.forced_response(G, t, u, 0.0)

fig = plt.figure('simulation', figsize=(10,5))
plt.clf()
ax = fig.add_subplot(1,1,1)

ax.plot(t, u, linestyle='--', label='PWM (percent)')
ax.plot(t, theta - theta_0, linestyle='-', label='Meas. temperature (minus offset)')
ax.plot(t, time_response.outputs, linestyle='-', label='Sim. temperature (minus offset)')
ax.legend()
ax.grid(True)

plt.show()



#
# The following sections/lines are not yet updated!
#

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



# %% Test the controller

s = ctrl.tf('s')
K = kP + kI/s

closed_loop = ctrl.feedback(ctrl.series(K, G))

t, h = ctrl.step_response(closed_loop)
plt.plot(t,h)
plt.grid(True)
plt.show()

K_disrete = ctrl.sample_system(K, 0.5)
print(K_disrete)
print(ctrl.ss(K_disrete))

# %% Own controller design

# Todo: check PWM signal limits
Kp = ctrl.tf([0.5], [1.0])
Ki = ctrl.tf([0.0010], [1.0, 0.0])
K = ctrl.parallel(Kp, Ki)

closed_loop = ctrl.feedback(ctrl.series(K, G))

t, h = ctrl.step_response(100*closed_loop)
plt.plot(t,h)
plt.xlim([0,720])
plt.grid(True)
plt.show()

# %% Get intermediate signals as well

G = ctrl.tf(G, inputs='u', outputs='y')
K = ctrl.tf(K, inputs='e', outputs='u')
sumblk = ctrl.summing_junction(inputs=['r', '-y'], output='e')
T = ctrl.interconnect([G, K, sumblk], inputs='r', outputs=['y', 'u', 'e'])

time_response = ctrl.step_response(T)
plt.plot(time_response.t, np.squeeze(time_response.y[0,...]), label='y')
plt.plot(time_response.t, np.squeeze(time_response.y[1,...]), label='u')
plt.plot(time_response.t, np.squeeze(time_response.y[2,...]), label='e')
plt.legend()
plt.xlim([0,720])
plt.grid(True)
plt.show()


# %% Reference signal from datasheet of the soldering paste

profile = {
    # time in seconds
    't':     [0,    60,   120,   180,   240,   300,   315,   330,   350,   355,   360,   420],
    # temperature in deg C
    'theta': [50.0, 75.0, 105.0, 140.0, 160.0, 220.0, 240.0, 245.0, 217.0, 200.0, 180.0, 50.0]
}

fig = plt.figure('Reference profile')
plt.clf()

ax = fig.add_subplot(1,1,1)
ax.plot(profile['t'], profile['theta'])
ax.grid(True)
ax.set(xlabel='t (s)')
ax.set(ylabel='theta (deg C)')
ax.set(xlim=(0,8*60))
ax.set(ylim=(0,270))

plt.show()

# %%
