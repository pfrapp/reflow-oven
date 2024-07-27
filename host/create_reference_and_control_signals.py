# %% Packages

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# %% Create signals

sample_time_ms = 500
max_runtime_seconds = 720

# Time signal
t = np.arange(0, max_runtime_seconds + sample_time_ms/1000, sample_time_ms/1000)
t_ms = 1000*t
t_ms = np.array(1000*t, dtype='int64')

# Create step control signal
# Step to 20%
u = np.array([20 if tau > 10.0 else 0.0 for tau in t])

# Create reference control signal
# 40 deg Celcius
w = np.array([40 if tau > 10.0 else 20.0 for tau in t])

# %% Plot

plt.figure(100)
plt.plot(t, u, label='control signal u')
plt.plot(t, w, label='reference signal w', linestyle='--')
plt.grid(True)
plt.xlabel('t (s)')
plt.legend()
plt.show()

# %% Create pandas dataframe

data = {
    'Time (ms)': t_ms,
    'Reference Temperature (C)': w,
    'Control (percent)': u
}
df = pd.DataFrame(data)

df.to_csv('reference_control_signals.log')




# %%
