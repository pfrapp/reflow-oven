# %%
# 
# Scope Messungen vom 15. Dez. 2024
# Cortex Kommunikation ueber I2C mit dem BMP280
# CH1 war SCL
# CH2 war SDA
#
import pandas as pd
import matplotlib.pyplot as plt


# %%

df = pd.read_csv('WFM01.csv')
print(df)

t = df['in s'].to_numpy()
scl = df['C1 in V'].to_numpy()
sda = df['C2 in V'].to_numpy()

# %%

plt.clf()
plt.plot(t, scl, label='SCL')
plt.plot(t, sda, label='SDA')
plt.grid(True)
plt.legend()
plt.show()

# %% Remove before t=0

scl, sda = scl[t>0], sda[t>0]
t = t[t>0]

# %% Zoom in
# Auffaellig: Die steigende Flanke ist relativ langsam.

N1 = 0
N2 = 1000

plt.clf()
plt.plot(t[N1:N2], scl[N1:N2], label='SCL')
plt.plot(t[N1:N2], sda[N1:N2], label='SDA')
plt.grid(True)
plt.legend()
plt.show()


# %%
