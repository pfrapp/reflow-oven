# %% Read measured signals

# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctrl

# %% Define the signals to be read

# 30% PWM signal
path_prefix = './../measurements/2024-07-27/'
meas_file = 'signals_step_30_percent.log'
signal_30 = path_prefix + meas_file

# 50% PWM signal
path_prefix = './../measurements/2024-07-27/'
meas_file = 'signals_step_50_percent.log'
signal_50 = path_prefix + meas_file

# 100% PWM signal with stop after 100 deg C
path_prefix = './../measurements/2024-08-03/'
meas_file = 'signals_step_100_percent_until_100_degC_cooling_down_door_closed.log'
signal_100_stop = path_prefix + meas_file


# %% Read and plot the signals

df = pd.read_csv(signal_100_stop)

fig = plt.figure(meas_file, figsize=(10,5))
plt.clf()

ax = fig.add_subplot(1,1,1)
ax.plot(df['Time (ms)'][1:]/1000, df['Temperature (C)'][1:],
         linestyle='-', label='Temperature (C)')
ax.plot(df['Time (ms)'][1:]/1000, df['pwm_controller (percent)'][1:],
         linestyle='--', label='PWM controller (percent)')
ax.grid(True)
ax.set(ylim=(0, 300))
ax.set(xlabel='t (ms)')
# ax.set(ylabel='Temperature (C)')
ax.set(title='Amplified thermocouple voltage')
ax.legend()

plt.show()

# %% Offset removal and fitting of PT1

t = df['Time (ms)'] / 1000.0
theta = df['Temperature (C)']
# Do not start with 0 due to a non-existing meaningful measurement there.
theta_0 = np.mean(theta[1:40])
print(f'theta_0 = {theta_0} deg C')

# Delay of the step function
step_delay = 30.0

# PWM percentage
pwm_percent = 100.0

pt1_delay_params = {
    'lag': 30.0,
    'amplitude': 250.0/50*pwm_percent,
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


# %% Simulation via plant model

# tf_pt1 = ctrl.tf([505.0], [320.0, 1.0]) # 50% pwm
tf_pt1 = ctrl.tf([550.0], [370.0, 1.0]) # 30% pwm
num, den = ctrl.pade(40, 5)
tf_Td = ctrl.tf(num, den)
G = ctrl.series(tf_Td, tf_pt1)

fig = plt.figure('simulation', figsize=(7,12))
plt.clf()
ax = fig.add_subplot(3,1,1)

t_step, h_step = ctrl.step_response(0.01*pwm_percent*G)
# The step in the measurement comes after 30 seconds
t_step += 30.0
ax.plot(t_step, h_step, linestyle='--', label='Simulation')
ax.plot(t[1:], theta[1:] - theta_0, linestyle='-', label='Measurement')
ax.set(xlim=(0,1500))
# plt.xlim(0,300)
# plt.ylim(-10,300)
ax.grid(True)

plt.show()

# %% Use the actual input signal

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
t = t[1:]

theta = df['Temperature (C)'].to_numpy()
# Do not start with 0 due to a non-existing meaningful measurement there.
theta = theta[1:]
theta_0 = np.mean(theta[:40])
print(f'theta_0 = {theta_0} deg C')

u = df['pwm_controller (percent)'].to_numpy()
u = u[1:]

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

# %% Validate the model


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
