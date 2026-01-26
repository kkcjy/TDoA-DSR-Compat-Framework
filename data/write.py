import re

input_file = "vicon.txt"
output_file_4 = "neighbor_4.txt"
output_file_12 = "neighbor_12.txt"

pattern = re.compile(
    r"\[local_TDoA_3\s*<-\s*neighbor_TDoA_(\d+)\]:\s*vicon dist\s*=\s*([-\d.]+),\s*time\s*=\s*(\d+)"
)

lines_4 = []
lines_12 = []

with open(input_file, "r") as f:
    for line in f:
        m = pattern.search(line)
        if not m:
            continue
        neighbor = int(m.group(1))
        if neighbor == 4:
            lines_4.append(line)
        elif neighbor in (1, 2):
            lines_12.append(line)

sampled_4 = lines_4[::17]    # 每 17 行取 1 行
sampled_12 = lines_12[::5]   # 每 5 行取 1 行

with open(output_file_4, "w") as f:
    f.writelines(sampled_4)

with open(output_file_12, "w") as f:
    f.writelines(sampled_12)

print(f"neighbor=4 original: {len(lines_4)}, sampled: {len(sampled_4)}")
print(f"neighbor=1/2 original: {len(lines_12)}, sampled: {len(sampled_12)}")
