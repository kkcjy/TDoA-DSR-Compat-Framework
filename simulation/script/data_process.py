import re
import numpy as np
import pandas as pd


# This script reads data collected by the sniffer and organizes it into a format suitable for simulation, saving the results in the simulation/data directory.


# Number of drones in the simulation.
DRONE_NUM = 3
# Determines whether to filter out incomplete communications.
INTEGRITY_FILTER = False


# get number of Txi and Rxi from the header of the sniffer data file by seq
def count_tx_rx_from_header(file_path):
    with open(file_path, 'r') as f:
        header_line = f.readline().strip()
    
    headers = header_line.split(',')
    
    tx_count = 0
    rx_count = 0
    tx_pattern = re.compile(r'Tx(\d+)_seq')
    rx_pattern = re.compile(r'Rx(\d+)_seq')
    
    for header in headers:
        tx_match = tx_pattern.match(header)
        rx_match = rx_pattern.match(header)
        
        if tx_match:
            tx_num = int(tx_match.group(1))
            tx_count = max(tx_count, tx_num + 1)
        elif rx_match:
            rx_num = int(rx_match.group(1))
            rx_count = max(rx_count, rx_num + 1)
    
    return tx_count, rx_count


if __name__ == '__main__':
    line_num = 3 * DRONE_NUM
    table_pos = 0

    print("Starting to Read and Integrate Sniffer-Collected Data...")

    sniffer_data_path = '../../sniffer/data/sniffer_Log.csv'
    processed_data_path = '../data/processed_Log.csv'

    tx_count, rx_count = count_tx_rx_from_header(sniffer_data_path)
    print(f"Detected {tx_count} Tx and {rx_count} Rx in the sniffer data.")

    system_time = np.zeros(line_num, dtype = np.uint64)
    src_addr = np.zeros(line_num, dtype = np.uint16)
    msg_seq = np.zeros(line_num, dtype = np.uint16)
    filter = np.zeros(line_num, dtype = np.uint16)
    Tx_time = np.zeros(line_num, dtype = np.uint64)
    Rx_addr = np.zeros((line_num, rx_count), dtype = np.uint16)
    Rx_time = np.zeros((line_num, rx_count), dtype = np.uint64)

    with open(processed_data_path, 'w') as out_file:
        out_file.write("system_time, src_addr,msg_seq,filter,Tx_time")
        for i in range(rx_count):
            out_file.write(f",Rx{i}_addr,Rx{i}_time")
        out_file.write('\n')

    processed_count = 0
    output_count = 0

    with open(sniffer_data_path, 'r') as f:
        f.readline()

        while True:
            line = f.readline().strip()
            if not line:
                break

            parts = line.split(',')
            processed_count += 1

            # system_time, src_addr,msg_seq,filter
            system_time[table_pos] = int(parts[0])
            src_addr[table_pos] = int(parts[1])
            msg_seq[table_pos] = int(parts[2])
            filter[table_pos] = int(parts[4])

            # Tx time
            last_Tx_time = int(parts[5])
            last_Tx_seq = int(parts[6])

            found = False
            # forward_part[0, table_pos - 1]
            for i in range(table_pos - 1, -1, -1):
                if src_addr[i] == src_addr[table_pos] and msg_seq[i] == last_Tx_seq:
                    Tx_time[i] = last_Tx_time
                    found = True
                    break
            # backward_part[table_pos + 1, line_num - 1]
            if not found:
                for i in range(line_num - 1, table_pos, -1):
                    if src_addr[i] == src_addr[table_pos] and msg_seq[i] == last_Tx_seq:
                        Tx_time[i] = last_Tx_time
                        found = True
                        break

            # Rx time
            for rx_idx in range(rx_count):
                last_Tx_addr = int(parts[5 + 2 * tx_count + 3 * rx_idx])
                last_Rx_time = int(parts[6 + 2 * tx_count + 3 * rx_idx])
                last_Rx_seq = int(parts[7 + 2 * tx_count + 3 * rx_idx])

                found = False
                # forward_part[0, table_pos - 1]
                for i in range(table_pos - 1, -1, -1):
                    if src_addr[i] == last_Tx_addr and msg_seq[i] == last_Rx_seq:
                        for j in range(rx_count):
                            if Rx_addr[i, j] == last_Tx_addr and Rx_time[i, j] == last_Rx_time:
                                found = True
                                break
                            elif Rx_addr[i, j] == 0:
                                Rx_addr[i, j] = src_addr[table_pos]
                                Rx_time[i, j] = last_Rx_time
                                found = True
                                break
                        if found:
                            break
                # backward_part[table_pos + 1, line_num - 1]
                if not found:
                    for i in range(line_num - 1, table_pos, -1):
                        if src_addr[i] == last_Tx_addr and msg_seq[i] == last_Rx_seq:
                            for j in range(rx_count):
                                if Rx_addr[i, j] == last_Tx_addr and Rx_time[i, j] == last_Rx_time:
                                    found = True
                                    break
                                elif Rx_addr[i, j] == 0:
                                    Rx_addr[i, j] = src_addr[table_pos]
                                    Rx_time[i, j] = last_Rx_time
                                    found = True
                                    break
                            if found:
                                break

            # write to file
            if (table_pos + 1) % DRONE_NUM == 0 and src_addr[2 * DRONE_NUM - 1] != 0:
                for i in range(DRONE_NUM):
                    current_pos = (table_pos + 1 + i) % line_num

                    # check if all fields are valid and write to file
                    if INTEGRITY_FILTER:
                        all_valid = True
                        if Tx_time[current_pos] == 0:
                            all_valid = False

                        if all_valid:
                            for j in range(rx_count):
                                if Rx_time[current_pos, j] == 0:
                                    all_valid = False
                                    break

                        if all_valid:
                            with open(processed_data_path, 'a') as out_file:
                                out_file.write(f"{system_time[current_pos]},{src_addr[current_pos]},{msg_seq[current_pos]},{filter[current_pos]},{Tx_time[current_pos]}")
                                for j in range(rx_count):
                                    out_file.write(f",{Rx_addr[current_pos, j]},{Rx_time[current_pos, j]}")
                                out_file.write('\n')
                            output_count += 1

                    # directly write the collected data
                    else:
                        with open(processed_data_path, 'a') as outfile:
                            if Tx_time[current_pos] == 0:
                                continue
                            outfile.write(f"{system_time[current_pos]},{src_addr[current_pos]},{msg_seq[current_pos]},{filter[current_pos]},{Tx_time[current_pos]}")
                            for j in range(rx_count):
                                outfile.write(f",{Rx_addr[current_pos, j]},{Rx_time[current_pos, j]}")
                            outfile.write('\n')
                        output_count += 1

                    # clean up
                    system_time[current_pos] = 0
                    src_addr[current_pos] = 0
                    msg_seq[current_pos] = 0
                    filter[current_pos] = 0
                    Tx_time[current_pos] = 0
                    for j in range(rx_count):
                        Rx_addr[current_pos, j] = 0
                        Rx_time[current_pos, j] = 0

            table_pos = (table_pos + 1) % line_num

    print(f"Processing completed! Processed {processed_count} lines of data, output {output_count} valid records")
    print(f"Output file: {processed_data_path}")