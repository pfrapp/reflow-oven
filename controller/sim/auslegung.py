# %% Auslegung Thermocouple Verstaerker
#
# Referenzen:
# - SLOA030A (TI)
# - Thermocouple Typ K Tabelle
# - https://www.mikrocontroller.net/topic/283206
# - https://www.ti.com/lit/ds/symlink/opa335.pdf Seite 9

# %% Packages

import numpy as np
import matplotlib.pyplot as plt

# %% Berechnung der Parameter der linearen Abbildung
# V_out = m * V_in + b

# Wir nutzen 2 Stufen.
# Stufe 1:
# V_in ist im Bereich -2 mV bis +18 mV
# V_out soll von 3 V bis 3.5 V gehen (Supply ist +12 V gegen Ground) 

# [ -2.0e-3   1 ] [ m ]   [ 3 ]
#                =
# [ 18.0e-3  1  ] [ b ]   [ 4 ]


A = np.array([[0, 1.0],
              [16.0e-3, 1.0]])
b = np.array([8.0, 8.05])

x = np.linalg.solve(A, b)

m, b = x

print('Stufe 1\n-----------')
print(f'm = {m}\nb = {b}')


# %% Berechnung der Widerstaende

R1 = 4.7e3
vref = 12.0


R2 = (m/(b/vref)) * R1


z = (R1+R2)/R2
RG = 4.7e3
RF = (m*z-1)*RG


print(f'R1 = {R1}')
print(f'R2 = {R2}')
print(f'RG = {RG}')
print(f'RF = {RF}')


print(f'Pin +: {R1/(R1+R2)*vref} V')
print(f'Pin -: {RG/(RG+RF)*12.0} V')


# %% Stufe 2

# Stufe 2:
# V_in ist im Bereich 3 V bis 3.5 V
# V_out soll von 3 V bis 9 V gehen (Supply ist +12 V gegen Ground) 

# [ 3   1 ] [ m ]   [ 3 ]
#                =
# [ 3.5   1 ] [ b ]   [ 9 ]


A = np.array([[8, 1.0],
              [8.05, 1.0]])
b = np.array([3.0, 4.0])

x = np.linalg.solve(A, b)

m, b = x

print('Stufe 2\n-----------')
print(f'm = {m}\nb = {b}')

# %% Berechnung der Widerstaende

RG = 4.7e3
RF = (m-1) * RG
vref = 12.0

# R2/(R1+R2) = z
z = abs(b) / vref * RG/RF

# R2 = (R1+R2)*z = R1*z + R2*z
# (1-z)*R2 = R1*z
# R1 = (1-z)/z * R2

R2 = 1.2e3
R1 = (1-z)/z * R2

print(f'R1 = {R1}')
print(f'R2 = {R2}')
print(f'RG = {RG}')
print(f'RF = {RF}')

R1_R2_parallel = 1/(1/R1 + 1/R2)
print(f'R1_R2_parallel = {R1_R2_parallel} (kleiner als RG={RG})')

# %% Spannunsteiler

# 9 V --> 3.3 V
x = 3.3 / 9.0

y = 4.7e3/(4.7e3+8.2e3)

# %% Aufloesung

N_res = 12

dv = 2.84 - 2.36
dv_adc = 3.3/(2**N_res)

steps = dv / dv_adc

250 / steps

# %% Stufe 3

# Eingangsbereich (0.5 bis 1.2 V) muss erst noch
# vermessen bzw. bestaetigt werden.


A = np.array([[0.5, 1.0],
              [1.2, 1.0]])
b = np.array([3.0, 9])

x = np.linalg.solve(A, b)

m, b = x

print('Stufe 1\n-----------')
print(f'm = {m}\nb = {b}')

# %% Berechnung der Widerstaende

RG = 4.7e3
RF = (m-1) * RG
vref = 12.0

# R2/(R1+R2) = z
z = abs(b) / vref * RG/RF

# R2 = (R1+R2)*z = R1*z + R2*z
# (1-z)*R2 = R1*z
# R1 = (1-z)/z * R2

R2 = 1.2e3
R1 = (1-z)/z * R2

print(f'R1 = {R1}')
print(f'R2 = {R2}')
print(f'RG = {RG}')
print(f'RF = {RF}')

R1_R2_parallel = 1/(1/R1 + 1/R2)
print(f'R1_R2_parallel = {R1_R2_parallel} (kleiner als RG={RG})')


