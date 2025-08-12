import pandas as pd
from tqdm import tqdm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
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

            if abs(dis_unit) < jitter_threshold or dis_Compensate * dis_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)


def ranging_plot(cdsr, dsr, sr, vicon):
    plt.figure(figsize=(12, 6))

    plt.plot(sr, color="#3787E1", label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr, color="#E9950F", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(cdsr, color="#DA6055", label='CDSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon, color="#1DA556", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)

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

    best_dtw = float('inf')
    best_params = (None, None)

    compensate_rate_range = np.arange(0, 1.01, 0.01)
    jitter_threshold_range = np.arange(0, 10.1, 0.1)

    total_iterations = len(compensate_rate_range) * len(jitter_threshold_range)

    with tqdm(total=total_iterations, desc="Searching best parameters") as pbar:
        for param_compensate in compensate_rate_range:
            for param_jitter in jitter_threshold_range:
                curcdsr = compensation_algorithm(dsr, param_compensate, param_jitter)

                curcdsr_vec = np.array([[x] for x in curcdsr])
                vicon_vec = np.array([[x] for x in vicon])

                dist, path = fastdtw(curcdsr_vec, vicon_vec, dist=euclidean)

                if dist < best_dtw:
                    best_dtw = dist
                    best_params = (param_compensate, param_jitter)
                    cdsr = curcdsr

                pbar.update(1)  # 每次迭代更新进度条

    print(f"Best parameters found: COMPENSATE_RATE = {best_params[0]:.2f}, JITTER_THRESHOLD = {best_params[1]:.2f}")
    print(f"Minimum DTW distance = {best_dtw:.4f}")
    ranging_plot(cdsr, dsr, sr, vicon)