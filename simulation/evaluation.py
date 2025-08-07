import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
matplotlib.use('TkAgg')

FILE_NAME = './data/ranging_Log.csv'

def ranging_plot():
    df = pd.read_csv(FILE_NAME)

    plt.figure(figsize=(12, 6))
    plt.plot(df['DSR'], label='DSR', marker='o')
    plt.plot(df['SR'], label='SR', marker='s')
    plt.plot(df['VICON'], label='VICON', marker='^')
    plt.title('Ranging Comparison Over Time')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def compute_error_metrics(predicted, ground_truth):
    ae = np.abs(predicted - ground_truth)
    mean_ae = np.mean(ae)
    max_ae = np.max(ae)
    rmse = np.sqrt(np.mean((predicted - ground_truth) **2))
    re = ae / (ground_truth + 1e-10)
    mean_re = np.mean(re) * 100
    
    return mean_ae, max_ae, rmse, mean_re


if __name__ == '__main__':
    df = pd.read_csv(FILE_NAME)
    df = df.dropna(subset=['DSR', 'SR', 'VICON'])

    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(df['DSR'].values, df['VICON'].values)

    mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(df['SR'].values, df['VICON'].values)

    print("==== Error Metrics ====")
    print(f"DSR: Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%")
    print(f"SR : Mean AE(平均绝对误差) = {mean_ae_sr:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr:.3f} cm, RMSE(均方根误差) = {rmse_sr:.3f} cm, MRE(平均相对误差) = {mre_sr:.3f}%")

    ranging_plot()