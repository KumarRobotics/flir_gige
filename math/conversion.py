from math import exp, log

def raw_to_celsius(S, B, F, O, R, T0):
    return (B / log(R / (S - O) + F) - T0)

def celsius_to_raw(t, B, F, O, R, T0):
    return ((R / exp(B / (t + T0)) - F) + O)

B = 1428.0
F = 1.0
O = 118.126
R = 377312.0
T0 = 273.15

celsius = [10, 20, 30, 40, 50]
raw = [];
celcius_converted = []

for t in celsius:
    raw.append(celsius_to_raw(t, B, F, O, R, T0))

for s in raw:
    celcius_converted.append(raw_to_celsius(s, B, F, O, R, T0))

for t in celcius_converted:
    print(t)
