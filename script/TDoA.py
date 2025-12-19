import re
import numpy as np
import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

sns.set(style="whitegrid")  # 背景美化

# 颜色定义
anchor1_color = "#1f77b4"
anchor2_color = "#ff7f0e"

def tdoa_plot(anchor1_data, anchor2_data):
    sys_time = np.arange(len(anchor1_data))  # 用 index 作为时间轴
    fig, ax1 = plt.subplots(figsize=(24, 14))

    # 主 Y 轴
    ax1.set_xlabel('Measurement Index', fontsize=40, labelpad=40)
    ax1.set_ylabel('TDoA Distance Difference', fontsize=40, labelpad=40)

    # 绘 anchor1
    l1, = ax1.plot(sys_time, anchor1_data, color=anchor1_color, linestyle='-', linewidth=4, marker='o', markersize=10, label='Anchor1=1')
    # 均值线
    mu1 = np.mean(anchor1_data)
    sigma1 = np.std(anchor1_data)
    ax1.hlines(mu1, 0, len(anchor1_data)-1, colors=anchor1_color, linestyles='--', linewidth=3)
    # ±σ 阴影
    ax1.fill_between(sys_time, mu1 - sigma1, mu1 + sigma1, color=anchor1_color, alpha=0.2)

    # 绘 anchor2
    l2, = ax1.plot(np.arange(len(anchor2_data)), anchor2_data, color=anchor2_color, linestyle='-', linewidth=4, marker='s', markersize=10, label='Anchor1=2')
    mu2 = np.mean(anchor2_data)
    sigma2 = np.std(anchor2_data)
    ax1.hlines(mu2, 0, len(anchor2_data)-1, colors=anchor2_color, linestyles='--', linewidth=3)
    ax1.fill_between(np.arange(len(anchor2_data)), mu2 - sigma2, mu2 + sigma2, color=anchor2_color, alpha=0.2)

    # 坐标轴样式
    ax1.tick_params(axis='x', labelsize=35)
    ax1.tick_params(axis='y', labelsize=35)
    ax1.grid(True, linestyle='--', alpha=0.3)

    # 右上角显示 μ/σ
    param_text = f"Anchor1=1: μ={mu1:.2f}, σ={sigma1:.2f}\nAnchor1=2: μ={mu2:.2f}, σ={sigma2:.2f}"
    ax1.text(0.98, 0.95, param_text, transform=ax1.transAxes, ha='right', va='top',
             fontsize=30, bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

    # 图例
    lines = [l1, l2]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, fontsize=30, loc='upper left', frameon=False, ncol=2, columnspacing=3)

    plt.tight_layout()
    plt.show()

# 解析 TDoA.log 文件
LOG_FILE = "TDoA.log"
pattern = re.compile(r"TDoA dist diff = ([\-0-9\.]+),\s*anchor1 = (\d+),\s*anchor2 = (\d+),\s*time = (\d+)")

tdoa_anchor1 = []
tdoa_anchor2 = []

with open(LOG_FILE, "r", encoding="utf-8") as f:
    for line in f:
        match = pattern.search(line)
        if not match:
            continue
        dist_diff = float(match.group(1))
        anchor1 = int(match.group(2))
        if anchor1 == 1:
            tdoa_anchor1.append(dist_diff)
        elif anchor1 == 2:
            tdoa_anchor2.append(dist_diff)

tdoa_anchor1 = np.array(tdoa_anchor1)
tdoa_anchor2 = np.array(tdoa_anchor2)

tdoa_plot(tdoa_anchor1, tdoa_anchor2)
