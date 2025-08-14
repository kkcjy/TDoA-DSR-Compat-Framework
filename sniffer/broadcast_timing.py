import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

uwb_max_timestamp=1099511627776
meter_per_tick=0.4691763978616*1e-2
ms_per_meter=1000/299792458
ms_per_tick=ms_per_meter * meter_per_tick

# Read the CSV file into a DataFrame
#df = pd.read_csv('data/2025-08-07-10-45-43.csv')
#interval = 50.0
df = pd.read_csv('src/deck/drivers/AdHocUWB/sniffer/data/sniffer_Log.csv')

interval = 216.5
# df = pd.read_csv('/Users/twinhorse/Career/project/aerialswarm/gitprojects/crazyflie-firmware-adhocuwb/src/deck/drivers/AdHocUWB/sniffer/data/data.csv')

# Extract columns into arrays (as lists)
sniffer_rx_ms = df['system_time']
src_addrs = df['src_addr']

base = sniffer_rx_ms[0]

x = np.array([0])
y = np.array([0])
basey=sniffer_rx_ms[0]
newy=0
for i in range(len(sniffer_rx_ms)-1):
    diff = sniffer_rx_ms[i+1]-sniffer_rx_ms[0]
    newy = math.floor((diff)/interval)
    newx = math.fmod((diff), interval)
    x=np.append(x,newx)
    y=np.append(y,newy)

for i in range(len(x)):
    print(f"({x[i]}, {y[i]}), {src_addrs[i]}")

for neighbor in np.unique(src_addrs):
    indices = np.where(src_addrs == neighbor)
    plt.scatter(x[indices], y[indices])

plt.show()