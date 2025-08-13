import re
import pandas as pd
from tqdm import tqdm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
matplotlib.use('TkAgg')


# This script reads data from ranging log.csv and adjusts the compensation coefficient appropriately to optimize ranging accuracy, make sure COMPENSATE_ENABLE is closed.


local_address = 2
neighbor_address = 3
vicon_path = "../data/output/vicon.txt"
threshold = 5


def read_log():
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
    
    data_path = '../data/output/ranging_Log.csv'
    data = pd.read_csv(data_path)

    data['DSR'] = pd.to_numeric(data['DSR'], errors='coerce')
    data['SR'] = pd.to_numeric(data['SR'], errors='coerce')
    data['VICON'] = pd.to_numeric(data['VICON'], errors='coerce')
    data['TIME'] = pd.to_numeric(data['TIME'], errors='coerce')

    data.dropna(inplace=True)

    dsr = data['DSR'].to_numpy(dtype=float)
    sr = data['SR'].to_numpy(dtype=float)
    vicon_sample = data['VICON'].to_numpy(dtype=float)
    time = data['TIME'].to_numpy(dtype=float)

    vicon, vicon_sys_time = read_vicon_Log()

    return dsr, sr, vicon_sample, time, vicon, vicon_sys_time

def compensation_algorithm(distance_List, compensate_rate, jitter_threshold):
    last_dis_Calculate = None
    dis_unit = None
    distance_List_Processed = []

    for i in range(len(distance_List)):
        dis_Calculate = distance_List[i]

        if last_dis_Calculate is None:
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        elif dis_unit is None:
            dis_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        else:
            dis_Compensate = dis_unit
            dis_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate

            if abs(dis_unit) > jitter_threshold or dis_Compensate * dis_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)

def evaluate_params_mae(dsr, sr, vicon_sample, time, vicon, vicon_sys_time):
    cdsr = []

    best_mae = float('inf')
    best_params = (None, None)

    compensate_rate_range = np.arange(0, 1.01, 0.01)
    jitter_threshold_range = np.arange(0, 10.1, 0.1)

    for param_compensate in compensate_rate_range:
        for param_jitter in jitter_threshold_range:
            curcdsr = compensation_algorithm(dsr, param_compensate, param_jitter)

            curcdsr_filter = []
            vicon_filter = []

            if len(curcdsr) == len(vicon_sample):
                for i in range(len(curcdsr)):
                    if abs(curcdsr[i] - vicon_sample[i]) < threshold:
                        curcdsr_filter.append(curcdsr[i])
                        vicon_filter.append(vicon_sample[i])
            else:
                print(f"Length mismatch: curcdsr={len(curcdsr)}, vicon={len(vicon_sample)}")  
                continue

            cur_mae = np.mean(np.abs(np.array(curcdsr_filter) - np.array(vicon_filter)))
            
            if cur_mae < best_mae:
                cdsr = curcdsr
                best_params = (param_compensate, param_jitter)
                best_mae = cur_mae

    print(f"Best parameters found: COMPENSATE_RATE = {best_params[0]:.2f}, JITTER_THRESHOLD = {best_params[1]:.2f}")

    ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time)

def set_param(COMPENSATE_RATE, JITTER_THRESHOLD, dsr, sr, time, vicon, vicon_sys_time):
    cdsr = compensation_algorithm(dsr, COMPENSATE_RATE, JITTER_THRESHOLD)
    
    ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time)

def ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time):
    plt.figure(figsize=(12, 6))

    plt.plot(time, sr, color="#3787E1", label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(time, dsr, color="#E9950F", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(time, cdsr, color="#DA6055", label='CDSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#1DA556", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)

    plt.title('Ranging Comparison Over Time')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    dsr, sr, vicon_sample, time, vicon, vicon_sys_time = read_log()

    COMPENSATE_RATE = 0.2
    JITTER_THRESHOLD = 100

    evaluate_params_mae(dsr, sr, vicon_sample, time, vicon, vicon_sys_time)

    # set_param(COMPENSATE_RATE, JITTER_THRESHOLD, dsr, sr, time, vicon, vicon_sys_time)