import re
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


# This script integrates the DSR and SR data generated in the simulation along with the original VICON data, and formats them for use in evaluation and optimization.


local_address = 1
neighbor_address = 4

vicon_path = "../data/output/vicon.txt"
dsr_path = "../data/output/dynamic_swarm_ranging.txt"
sr_path = "../data/output/swarm_ranging.txt"


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

    return dsr_value, dsr_time


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

    return sr_value, sr_time

def filter_common_time(sr, sr_time, dsr, dsr_time):
    common_times = set(sr_time) & set(dsr_time)
    
    sr_filtered = []
    sr_time_filtered = []
    for s, t in zip(sr, sr_time):
        if t in common_times:
            sr_filtered.append(s)
            sr_time_filtered.append(t)

    dsr_filtered = []
    dsr_time_filtered = []
    for d, t in zip(dsr, dsr_time):
        if t in common_times:
            dsr_filtered.append(d)
            dsr_time_filtered.append(t)

    return sr_filtered, sr_time_filtered, dsr_filtered, dsr_time_filtered

def plot_sr_dsr(sr, sr_time, dsr, dsr_time):
    max_timestamp = max(max(sr_time), max(dsr_time)) + 1
    
    def correct_timestamps(timestamps, max_timestamp):
        corrected = []
        prev = timestamps[0]
        offset = 0
        for t in timestamps:
            if t < prev:
                offset += max_timestamp
            corrected.append(t + offset)
            prev = t
        return corrected

    sr_time_corrected = correct_timestamps(sr_time, max_timestamp)
    dsr_time_corrected = correct_timestamps(dsr_time, max_timestamp)

    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 6))
    plt.plot(sr_time_corrected, sr, label='SR', marker='o')
    plt.plot(dsr_time_corrected, dsr, label='DSR', marker='s')
    plt.xlabel('Corrected Timestamp')
    plt.ylabel('Distance')
    plt.title('SR and DSR over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    sr, sr_time = read_sr_Log()
    dsr, dsr_time = read_dsr_Log()

    sr, sr_time, dsr, dsr_time = filter_common_time(sr, sr_time, dsr, dsr_time)

    plot_sr_dsr(sr, sr_time, dsr, dsr_time)