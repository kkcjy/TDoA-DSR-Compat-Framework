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
# ---temp
# ranging_Log_path = '../data/output/ranging_Log.csv'
# vicon_path = "../data/output/vicon.txt"
# ---processed
file_num = "1"
ranging_Log_path = "../../../../../../data/processed/" + file_num + ".csv"
vicon_path = "../../../../../../data/processed/" + file_num + ".txt"
# ---packed loss
# csv_num = "1_30"
# txt_num = "1"
# ranging_Log_path = "../../../../../../data/packedloss/" + csv_num + ".csv"
# vicon_path = "../../../../../../data/packedloss/" + txt_num + ".txt"
# ---period
# csv_num = "1_50"
# txt_num = "1"
# ranging_Log_path = "../../../../../../data/period/" + csv_num + ".csv"
# vicon_path = "../../../../../../data/period/" + txt_num + ".txt"


def read_log():  
    def read_vicon_Log(): 
        vicon_value = []
        vicon_time = []
        pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: vicon dist = (-?\d+\.\d+), time = (\d+)")
        
        with open(vicon_path, "r", encoding="utf-8") as f:
            for line in f:
                if (match := pattern.search(line)):
                    vicon_value.append(float(match.group(1)))
                    vicon_time.append(int(match.group(2)))
        
        return vicon_value, vicon_time
    
    data = pd.read_csv(ranging_Log_path)
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

def evaluation_data(cdsr, dsr, sr, vicon):
    def compute_error_metrics(predicted, ground_truth):
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) **2))
        mean_re = np.mean(np.abs(predicted - ground_truth) / ground_truth) * 100
        return mean_ae, max_ae, rmse, mean_re
    
    if len(cdsr) == len(dsr) == len(sr) == len(vicon):
        mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(sr, vicon)
        mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr, vicon)
        mean_ae_cdsr, max_ae_cdsr, rmse_cdsr, mre_cdsr = compute_error_metrics(cdsr, vicon)

        print("==== Error Metrics ====")
        print(f"SR : Mean AE(平均绝对误差) = {mean_ae_sr:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr:.3f} cm, RMSE(均方根误差) = {rmse_sr:.3f} cm, MRE(平均相对误差) = {mre_sr:.3f}%")
        print(f"DSR: Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%")
        print(f"CDSR: Mean AE(平均绝对误差) = {mean_ae_cdsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_cdsr:.3f} cm, RMSE(均方根误差) = {rmse_cdsr:.3f} cm, MRE(平均相对误差) = {mre_cdsr:.3f}%")
    else:
        print(f"len(cdsr)={len(cdsr)}, len(dsr)={len(dsr)}, len(sr)={len(sr)}, len(vicon)={len(vicon)}")

def compensation_algorithm(distance_List, compensate_rate, deceleration_bound):
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

            if abs(dis_Compensate) - abs(dis_unit) > deceleration_bound or dis_Compensate * dis_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)

def ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time):
    plt.figure(figsize=(12, 6))
    plt.plot(time, sr, color="#4A90E2", label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(time, dsr, color="#E4491E", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(time, cdsr, color="#FF7B00", label='CDSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)
    plt.title('Ranging Comparison Over Time')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def set_param(COMPENSATE_RATE, DECELERATION_BOUND, dsr, sr, vicon_sample, time, vicon, vicon_sys_time):
    cdsr = compensation_algorithm(dsr, COMPENSATE_RATE, DECELERATION_BOUND)
    evaluation_data(cdsr, dsr, sr, vicon_sample)
    ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time)

def static_evaluate_params(dsr, sr, vicon_sample, time, vicon, vicon_sys_time):
    cdsr = []
    best_mae = float('inf')
    best_params = (None, None)
    compensate_rate_range = np.arange(0, 1.01, 0.01)
    deceleration_bound_range = np.arange(0, 20.1, 0.1)
    total_iterations = len(compensate_rate_range) * len(deceleration_bound_range)

    with tqdm(total=total_iterations, desc="Evaluating parameters", ncols=100) as pbar:
        for param_compensate in compensate_rate_range:
            for param_deceleration in deceleration_bound_range:
                curcdsr = compensation_algorithm(dsr, param_compensate, param_deceleration)
                cur_mae = np.mean(np.abs(np.array(curcdsr) - np.array(vicon_sample)))
                if cur_mae < best_mae:
                    cdsr = curcdsr
                    best_params = (param_compensate, param_deceleration)
                    best_mae = cur_mae
                pbar.update(1)  

    print(f"\nBest parameters found: COMPENSATE_RATE = {best_params[0]:.2f}, DECELERATION_BOUND = {best_params[1]:.2f}")

    evaluation_data(cdsr, dsr, sr, vicon_sample)
    ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time)

def dynamic_evaluate_params(dsr, sr, vicon_sample, time, vicon, vicon_sys_time):
    cdsr = []

    

    evaluation_data(cdsr, dsr, sr, vicon_sample)
    ranging_plot(cdsr, dsr, sr, time, vicon, vicon_sys_time)


if __name__ == '__main__':
    dsr, sr, vicon_sample, time, vicon, vicon_sys_time = read_log()

    # COMPENSATE_RATE = 0.2
    # DECELERATION_BOUND = 5
    # set_param(COMPENSATE_RATE, DECELERATION_BOUND, dsr, sr, vicon_sample, time, vicon, vicon_sys_time)

    static_evaluate_params(dsr, sr, vicon_sample, time, vicon, vicon_sys_time)