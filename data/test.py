import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


def moving_average_filter(x, win):
    win = int(win)
    if len(x) < win:
        return np.array([])
    kernel = np.ones(win) / win
    return np.convolve(x, kernel, mode="same")

def read_segments(log_file):
    tdoa_pattern = re.compile(r"TDoA dist diff\s*=\s*([-\d.]+),\s*anchor1\s*=\s*(\d+),\s*anchor2\s*=\s*(\d+)")
    dsr_pattern = re.compile(r"\[local_(\d+)\s*<-\s*neighbor_(\d+)\]:\s*CDSR dist\s*=\s*([-\d.]+)")
    dsr_phase1, dsr_phase2 = [], []
    tdoa_12_phase1, tdoa_12_phase2 = [], []
    tdoa_21_phase1, tdoa_21_phase2 = [], []
    phase = 1
    current_type = "DSR"

    with open(log_file, "r") as f:
        for line in f:
            if dsr_pattern.search(line):
                if current_type != "DSR":
                    phase = 2
                    current_type = "DSR"
                dist = float(dsr_pattern.search(line).group(3))
                if phase == 1:
                    dsr_phase1.append(dist)
                else:
                    dsr_phase2.append(dist)

            elif tdoa_pattern.search(line):
                if current_type != "TDoA":
                    current_type = "TDoA"
                m = tdoa_pattern.search(line)
                dist = float(m.group(1))
                a1 = int(m.group(2))
                a2 = int(m.group(3))
                if a1 == 1 and a2 == 2:
                    if phase == 1:
                        tdoa_12_phase1.append(dist)
                    else:
                        tdoa_12_phase2.append(dist)
                elif a1 == 2 and a2 == 1:
                    if phase == 1:
                        tdoa_21_phase1.append(dist)
                    else:
                        tdoa_21_phase2.append(dist)

    return (np.array(dsr_phase1), np.array(tdoa_12_phase1), np.array(tdoa_21_phase1),
            np.array(dsr_phase2), np.array(tdoa_12_phase2), np.array(tdoa_21_phase2))

def read_vicon_dsr(file_path):
    pattern = re.compile(r"\[local_TDoA_3\s*<-\s*neighbor_TDoA_4\]:\s*vicon dist\s*=\s*([-\d.]+),\s*time\s*=\s*(\d+)")
    distances = []

    with open(file_path, "r") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                distances.append(float(m.group(1)))
    return np.array(distances)

def read_vicon_tdoa(file_path):
    pattern_1 = re.compile(r"\[local_TDoA_3\s*<-\s*neighbor_TDoA_1\]:\s*vicon dist\s*=\s*([-\d.]+),\s*time\s*=\s*(\d+)")
    pattern_2 = re.compile(r"\[local_TDoA_3\s*<-\s*neighbor_TDoA_2\]:\s*vicon dist\s*=\s*([-\d.]+),\s*time\s*=\s*(\d+)")
    distances_1 = []
    distances_2 = []

    with open(file_path, "r") as f:
        for line in f:
            m1 = pattern_1.search(line)
            if m1:
                distances_1.append(float(m1.group(1)))
                continue
            m2 = pattern_2.search(line)
            if m2:
                distances_2.append(float(m2.group(1)))
    arr1 = np.array(distances_1)
    arr2 = np.array(distances_2)
    return arr1 - arr2, arr2 - arr1

def plot_global(dsr_p1, dsr_p2, filt_12_p1, filt_21_p1, filt_12_p2, filt_21_p2, vicon_dsr_1, vicon_dsr_2, vicon_tdoa1_1, vicon_tdoa2_1, vicon_tdoa1_2, vicon_tdoa2_2):
    idx = 0
    idx_dsr_p1 = np.arange(idx, idx + len(dsr_p1))
    idx = idx_dsr_p1[-1] + 1

    idx_t12_p1 = np.arange(idx, idx + len(filt_12_p1))
    idx_t21_p1 = np.arange(idx, idx + len(filt_21_p1))
    idx = max(idx_t12_p1[-1], idx_t21_p1[-1]) + 1

    idx_dsr_p2 = np.arange(idx, idx + len(dsr_p2))
    idx = idx_dsr_p2[-1] + 1

    idx_t12_p2 = np.arange(idx, idx + len(filt_12_p2))
    idx_t21_p2 = np.arange(idx, idx + len(filt_21_p2))

    plt.figure(figsize=(14, 6))

    diff_dsr = 17.5
    diff_tdoa = 7.5

    plt.plot(idx_dsr_p1, dsr_p1 + diff_dsr, "-", label="DSR", color="#fdb185", linewidth = 2.2)
    plt.plot(idx_dsr_p2, dsr_p2 + diff_dsr, "-", label="_nolegend_", color="#fdb185", linewidth = 2.2)

    plt.plot(idx_t12_p1, filt_12_p1 + diff_tdoa, "-", label="TDoA 1", color="#84C4FD", linewidth = 2.2)
    plt.plot(idx_t21_p1, filt_21_p1 + diff_tdoa, "-", label="TDoA 2", color="#94D6B2", linewidth = 2.2)
    plt.plot(idx_t12_p2, filt_12_p2 + diff_tdoa, "-", label="_nolegend_", color="#84C4FD", linewidth = 2.2)
    plt.plot(idx_t21_p2, filt_21_p2 + diff_tdoa, "-", label="_nolegend_", color="#94D6B2", linewidth = 2.2)

    idx_vicon_dsr_1 = np.arange(len(vicon_dsr_1)) - 5
    idx_vicon_dsr_2 = np.arange(len(vicon_dsr_2)) + idx_dsr_p2[0]
    plt.plot(idx_vicon_dsr_1, vicon_dsr_1, "--", label="Vicon DSR", color="#849ae8", alpha=0.9, linewidth = 2)
    plt.plot(idx_vicon_dsr_2, vicon_dsr_2, "--", label="_nolegend_", color="#849ae8", alpha=0.9, linewidth = 2)

    idx_vicon_tdoa1_1 = np.arange(len(vicon_tdoa1_1)) + idx_t12_p1[0]
    idx_vicon_tdoa2_1 = np.arange(len(vicon_tdoa2_1)) + idx_t21_p1[0]
    idx_vicon_tdoa1_2 = np.arange(len(vicon_tdoa1_2)) + idx_t12_p2[0]
    idx_vicon_tdoa2_2 = np.arange(len(vicon_tdoa2_2)) + idx_t21_p2[0]

    plt.plot(idx_vicon_tdoa1_1, vicon_tdoa1_1, "--", label="Vicon TDoA 1", color="#F5C167", alpha=0.9, linewidth = 2)
    plt.plot(idx_vicon_tdoa2_1, vicon_tdoa2_1, "--", label="Vicon TDoA 2", color="#ff9896", alpha=0.9, linewidth = 2)
    plt.plot(idx_vicon_tdoa1_2, vicon_tdoa1_2, "--", label="_nolegend_", color="#F5C167", alpha=0.9, linewidth = 2)
    plt.plot(idx_vicon_tdoa2_2, vicon_tdoa2_2, "--", label="_nolegend_", color="#ff9896", alpha=0.9, linewidth = 2)

    plt.xlabel("Index", fontsize=15)
    plt.ylabel("Distance / TDoA dist diff (cm)", fontsize=15)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.grid(True, alpha=0.5, linewidth=0.8)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.show()

def plot_global_with_raw(dsr_p1, tdoa_12_p1, tdoa_21_p1, dsr_p2, tdoa_12_p2, tdoa_21_p2, filt_12_p1, filt_21_p1, filt_12_p2, filt_21_p2):
    idx = 0
    idx_dsr_p1 = np.arange(idx, idx + len(dsr_p1))
    idx = idx_dsr_p1[-1] + 1

    idx_t12_p1 = np.arange(idx, idx + len(tdoa_12_p1))
    idx_t21_p1 = np.arange(idx, idx + len(tdoa_21_p1))
    idx = max(idx_t12_p1[-1], idx_t21_p1[-1]) + 1

    idx_dsr_p2 = np.arange(idx, idx + len(dsr_p2))
    idx = idx_dsr_p2[-1] + 1

    idx_t12_p2 = np.arange(idx, idx + len(tdoa_12_p2))
    idx_t21_p2 = np.arange(idx, idx + len(tdoa_21_p2))

    plt.figure(figsize=(14, 6))

    plt.plot(idx_dsr_p1, dsr_p1, "-", linewidth=2, label="DSR Phase1")
    plt.plot(idx_dsr_p2, dsr_p2, "-", linewidth=2, label="DSR Phase2")

    plt.plot(idx_t12_p1, tdoa_12_p1, "--", alpha=0.4, label="Raw TDoA 1->2 Phase1")
    plt.plot(idx_t21_p1, tdoa_21_p1, "--", alpha=0.4, label="Raw TDoA 2->1 Phase1")
    plt.plot(idx_t12_p2, tdoa_12_p2, "--", alpha=0.4, label="Raw TDoA 1->2 Phase2")
    plt.plot(idx_t21_p2, tdoa_21_p2, "--", alpha=0.4, label="Raw TDoA 2->1 Phase2")

    plt.plot(idx_t12_p1, filt_12_p1, "-", linewidth=2, label="MA TDoA 1->2 Phase1")
    plt.plot(idx_t21_p1, filt_21_p1, "-", linewidth=2, label="MA TDoA 2->1 Phase1")
    plt.plot(idx_t12_p2, filt_12_p2, "-", linewidth=2, label="MA TDoA 1->2 Phase2")
    plt.plot(idx_t21_p2, filt_21_p2, "-", linewidth=2, label="MA TDoA 2->1 Phase2")

    plt.xlabel("Global Sample Index")
    plt.ylabel("Distance / TDoA dist diff")
    plt.title("DSR and TDoA over All Phases")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    log_file = "log.txt"
    file_dsr_path_1 = "neighbor_4_1.txt"
    file_dsr_path_2 = "neighbor_4_2.txt"
    file_tdoa_path_1 = "neighbor_12_1.txt"
    file_tdoa_path_2 = "neighbor_12_2.txt"
    win = 75

    dsr_p1, tdoa_12_p1, tdoa_21_p1, dsr_p2, tdoa_12_p2, tdoa_21_p2 = read_segments(log_file)
    vicon_dsr_1 = read_vicon_dsr(file_dsr_path_1)
    vicon_dsr_2 = read_vicon_dsr(file_dsr_path_2)
    vicon_tdoa1_1, vicon_tdoa2_1 = read_vicon_tdoa(file_tdoa_path_1)
    vicon_tdoa1_2, vicon_tdoa2_2 = read_vicon_tdoa(file_tdoa_path_2)

    print("[Phase1] DSR:", len(dsr_p1), "TDoA 1->2:", len(tdoa_12_p1), "TDoA 2->1:", len(tdoa_21_p1))
    print("[Phase2] DSR:", len(dsr_p2), "TDoA 1->2:", len(tdoa_12_p2), "TDoA 2->1:", len(tdoa_21_p2))

    filt_12_p1 = moving_average_filter(tdoa_12_p1, win)
    filt_21_p1 = moving_average_filter(tdoa_21_p1, win)
    filt_12_p2 = moving_average_filter(tdoa_12_p2, win)
    filt_21_p2 = moving_average_filter(tdoa_21_p2, win)

    plot_global(dsr_p1, dsr_p2, filt_12_p1, filt_21_p1, filt_12_p2, filt_21_p2, vicon_dsr_1, vicon_dsr_2, vicon_tdoa1_1, vicon_tdoa2_1, vicon_tdoa1_2, vicon_tdoa2_2)
    # plot_global_with_raw(dsr_p1, tdoa_12_p1, tdoa_21_p1, dsr_p2, tdoa_12_p2, tdoa_21_p2, filt_12_p1, filt_21_p1, filt_12_p2, filt_21_p2)
