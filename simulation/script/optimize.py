import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
matplotlib.use('TkAgg')


# This script reads data from ranging log.csv and adjusts the compensation coefficient appropriately to optimize ranging accuracy, make sure COMPENSATE_ENABLE is closed.


def read_log():
    data_path = '../data/output/ranging_Log.csv'
    data = pd.read_csv(data_path)

    data['DSR'] = pd.to_numeric(data['DSR'], errors='coerce')
    data['SR'] = pd.to_numeric(data['SR'], errors='coerce')
    data['VICON'] = pd.to_numeric(data['VICON'], errors='coerce')

    data.dropna(inplace=True)

    dsr = data['DSR'].to_numpy(dtype=float)
    sr = data['SR'].to_numpy(dtype=float)
    vicon = data['VICON'].to_numpy(dtype=float)

    return dsr, sr, vicon

def compensation_algorithm(distance_List, compensate_rate, jitter_threshold):
    NULL_DIS = None

    last_dis_Calculate = NULL_DIS
    dis_unit = NULL_DIS
    distance_List_Processed = []

    for i in range(len(distance_List)):
        dis_Calculate = distance_List[i]
        distance = NULL_DIS

        # Compensation Algorithm (Not necessary to consider SEQGAP_THRESHOLD)
        if last_dis_Calculate == NULL_DIS:
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        elif dis_unit == NULL_DIS:
            dis_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        else:
            dis_Compensate = dis_unit
            dis_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate
            if abs(dis_unit) < jitter_threshold or dis_Compensate * dis_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)

def ranging_plot(cdsr, dsr, sr, vicon):
    plt.figure(figsize=(12, 6))

    plt.plot(cdsr, label='CDSR', marker='x')
    plt.plot(dsr, label='DSR', marker='o')
    plt.plot(sr, label='SR', marker='s')
    plt.plot(vicon, label='VICON', marker='^')

    plt.title('Ranging Comparison Over Time')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    dsr, sr, vicon = read_log()
    cdsr = []

    best_mae = float('inf')
    best_params = (None, None)

    compensate_rate_range = np.arange(0, 1.01, 0.01)
    jitter_threshold_range = np.arange(0, 10.1, 0.1)

    for param_compensate in compensate_rate_range:
        for param_jitter in jitter_threshold_range:
            curcdsr = compensation_algorithm(dsr, param_compensate, param_jitter)

            mae = np.mean(np.abs(curcdsr - vicon))

            if mae < best_mae:
                best_mae = mae
                best_params = (param_compensate, param_jitter)
                cdsr = curcdsr

    print(f"COMPENSATE_RATE = {best_params[0]:.2f}, JITTER_THRESHOLD = {best_params[1]:.2f}")
    ranging_plot(cdsr, dsr, sr, vicon)