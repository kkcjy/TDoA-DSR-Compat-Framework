import re
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


# This script integrates the DSR and SR data generated in the simulation along with the original VICON data, and formats them for use in evaluation and optimization.


# Set to the required address
local_address = 2
neighbor_address = 3

sys_path = "../data/processed_Log.csv"
dsr_path = "../data/output/dynamic_swarm_ranging.txt"
sr_path = "../data/output/swarm_ranging.txt"
vicon_path = "../data/output/vicon.txt"


def align_time_with_sys(time_list):
    sys_time = []
    rx_time = []
    align_sys_time = []

    with open(sys_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            sys_time.append(int(row['system_time']))
            rx_time.append(int(row['Rx0_time']))

    index = 0
    length = len(time_list)

    for i in range(len(rx_time)):
        if index >= length:
            break
        if rx_time[i] == time_list[index]:
            align_sys_time.append(sys_time[i])
            index += 1
        
    return align_sys_time

def read_vicon_Log():
    vicon_value = []
    vicon_time = []

    pattern = re.compile(
        rf"\[local_(?:{local_address}|{neighbor_address}) <- neighbor_(?:{neighbor_address}|{local_address})\]: vicon dist = (-?\d+\.\d+), time = (\d+)"
    )

    with open(vicon_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                vicon_val = float(match.group(1)) 
                ts_val = int(match.group(2))   
                vicon_value.append(vicon_val)
                vicon_time.append(ts_val)      


    return vicon_value, vicon_time

def read_dsr_Log():
    dsr_value = []
    dsr_time = []

    pattern = re.compile(
        rf"\[local_(?:{local_address}|{neighbor_address}) <- neighbor_(?:{neighbor_address}|{local_address})\]: DSR dist = (-?\d+\.\d+), time = (\d+)"
    )

    with open(dsr_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                dsr_val = float(match.group(1)) 
                ts_val = int(match.group(2))   
                dsr_value.append(dsr_val)
                dsr_time.append(ts_val)      

    dsr_sys_time = align_time_with_sys(dsr_time)

    return dsr_value, dsr_time, dsr_sys_time

def read_sr_Log():
    sr_value = []
    sr_time = []

    pattern = re.compile(
        rf"\[local_(?:{local_address}|{neighbor_address}) <- neighbor_(?:{neighbor_address}|{local_address})\]: SR dist = (-?\d+), time = (\d+)"
    )

    with open(sr_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                sr_val = int(match.group(1))
                ts_val = int(match.group(2))
                sr_value.append(sr_val)
                sr_time.append(ts_val)

    sr_sys_time = align_time_with_sys(sr_time)

    return sr_value, sr_time, sr_sys_time

def plot_sr_dsr_vicon(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    plt.plot(sr_sys_time, sr, color='#4A90E2', label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr_sys_time, dsr, color='#F5A623', label='DSR Midpoints', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color='#27AE60', label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)

    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title('SR vs DSR vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

def plot_sr_dsr_mid_vicon(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    dsr_sys_time = np.array(dsr_sys_time)
    dsr = np.array(dsr)

    dsr_mid_x = (dsr_sys_time[:-1] + dsr_sys_time[1:]) / 2
    dsr_mid_y = (dsr[:-1] + dsr[1:]) / 2

    plt.plot(sr_sys_time, sr, color='#4A90E2', label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr_mid_x, dsr_mid_y, color='#F5A623', label='DSR Midpoints', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color='#27AE60', label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)

    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title('SR vs Mid-Point DSR vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    sr, sr_time, sr_sys_time = read_sr_Log()
    dsr, dsr_time, dsr_sys_time = read_dsr_Log()
    vicon, vicon_sys_time = read_vicon_Log()

    # plot_sr_dsr_vicon(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)
    
    plot_sr_dsr_mid_vicon(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)