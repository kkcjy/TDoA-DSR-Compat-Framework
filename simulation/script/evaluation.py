import re
import csv
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


# This script integrates the processed SR and DSR data, aligns them with the VICON timestamps, and then evaluates the data.


# Set to the required address
local_address = 2
neighbor_address = 3
time_threshold = 10
leftbound = 1410370
rightbound = 1422430

sys_path = "../data/processed_Log.csv"
dsr_path = "../data/output/dynamic_swarm_ranging.txt"
sr_path = "../data/output/swarm_ranging.txt"
vicon_path = "../data/output/vicon.txt"
ranging_Log_path = "../data/output/ranging_Log.csv"


def align_sys_time(time_list):
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

    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: vicon dist = (-?\d+\.\d+), time = (\d+)")

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

    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: DSR dist = (-?\d+\.\d+), time = (\d+)")

    with open(dsr_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                dsr_val = float(match.group(1)) 
                ts_val = int(match.group(2))   
                dsr_value.append(dsr_val)
                dsr_time.append(ts_val)      

    dsr_sys_time = align_sys_time(dsr_time)

    return dsr_value, dsr_time, dsr_sys_time

def read_sr_Log():
    sr_value = []
    sr_time = []

    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: SR dist = (-?\d+), time = (\d+)")

    with open(sr_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                sr_val = int(match.group(1))
                ts_val = int(match.group(2))
                sr_value.append(sr_val)
                sr_time.append(ts_val)

    sr_sys_time = align_sys_time(sr_time)

    return sr_value, sr_time, sr_sys_time

def write_ranging_Log(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    def get_diff_dis(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
        left_idx_sr = np.searchsorted(sr_sys_time, leftbound, side='left')
        right_idx_sr = np.searchsorted(sr_sys_time, rightbound, side='right') - 1
        sr_slice = sr[left_idx_sr : right_idx_sr]
        sr_mean = np.mean(sr_slice)

        left_idx_dsr = np.searchsorted(dsr_sys_time, leftbound, side='left')
        right_idx_dsr = np.searchsorted(dsr_sys_time, rightbound, side='right') - 1
        dsr_slice = dsr[left_idx_dsr : right_idx_dsr]
        dsr_mean = np.mean(dsr_slice)

        left_idx_vicon = np.searchsorted(vicon_sys_time, leftbound, side='left')
        right_idx_vicon = np.searchsorted(vicon_sys_time, rightbound, side='right') - 1
        vicon_slice = vicon[left_idx_vicon : right_idx_vicon]
        vicon_mean = np.mean(vicon_slice)

        sr_diff = vicon_mean - sr_mean
        dsr_diff = vicon_mean - dsr_mean

        return sr_diff, dsr_diff

    sr_sys_time = np.array(sr_sys_time)
    sr = np.array(sr)
    dsr_sys_time = np.array(dsr_sys_time)
    dsr = np.array(dsr)
    vicon_sys_time = np.array(vicon_sys_time)
    vicon = np.array(vicon)

    sr_diff, dsr_diff = get_diff_dis(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    align_sr = sr + sr_diff
    align_dsr = dsr + dsr_diff
    align_vicon = vicon

    if len(sr_sys_time) == len(dsr_sys_time) and np.all(sr_sys_time == dsr_sys_time):
        with open(ranging_Log_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["DSR", "SR", "VICON", "TIME"])

            count = 0
            for i in range(len(sr_sys_time)):
                t = sr_sys_time[i]
                idx = np.argmin(np.abs(vicon_sys_time - t))
                time_diff = abs(vicon_sys_time[idx] - t)

                if time_diff > time_threshold:
                    continue 
                
                writer.writerow([align_dsr[i], align_sr[i], align_vicon[idx], sr_sys_time[i]])
                count += 1

        print(f"Ranging log saved to {ranging_Log_path}, total {count} records.")
    else:
        print("Error: sr_sys_time and dsr_sys_time are not identical. Cannot write log.")

    return align_sr, align_dsr, align_vicon

def plot_sr_dsr_vicon(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    plt.plot(sr_sys_time, sr, color='#4A90E2', label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr_sys_time, dsr, color="#E4491E", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)

    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title('SR vs DSR vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

def evaluation_data():
    def compute_error_metrics(predicted, ground_truth):
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) **2))
        re = ae / (ground_truth + 1e-10)
        mean_re = np.mean(re) * 100
        return mean_ae, max_ae, rmse, mean_re

    df = pd.read_csv(ranging_Log_path)
    df = df.dropna(subset=['DSR', 'SR', 'VICON'])

    mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(df['SR'].values, df['VICON'].values)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(df['DSR'].values, df['VICON'].values)

    print("==== Error Metrics ====")
    print(f"SR : Mean AE(平均绝对误差) = {mean_ae_sr:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr:.3f} cm, RMSE(均方根误差) = {rmse_sr:.3f} cm, MRE(平均相对误差) = {mre_sr:.3f}%")
    print(f"DSR: Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%")


if __name__ == '__main__':
    sr, sr_time, sr_sys_time = read_sr_Log()
    dsr, dsr_time, dsr_sys_time = read_dsr_Log()
    vicon, vicon_sys_time = read_vicon_Log()

    align_sr, align_dsr, align_vicon = write_ranging_Log(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    evaluation_data()

    plot_sr_dsr_vicon(align_sr, sr_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time)