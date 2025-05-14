import pickle
import struct

import pandas as pd
import numpy as np

DW_TIME_TO_MS = (1.0 / 499.2e6 / 128.0) * 1000  # 将dw3000时间戳转换为ms
PERIOD = 5  # 周期为ms
offset = 0
ADDRESS = 'data/2025-05-14-14-43-51.pkl'
with open(ADDRESS, 'rb') as file:
    data = pickle.load(file)

data = pd.DataFrame(data)


def process_overflow(row):
    '''
    处理溢出问题，使得时间点连续
    :param row:
    :return:
    '''
    global offset
    if row['diff'] < 0:
        offset += (0x10000000000 * DW_TIME_TO_MS)
    row['sniffer_rx_time'] += offset + PERIOD
    return row


def parse_body_unit(data):
    ''' 
    typedef union{
      struct {
        uint8_t rawtime[5];//低5位字节
        uint8_t address; //最高1位字节
        uint16_t seqNumber; //最高2-3位字节
      }__attribute__((packed));
      dwTime_t timestamp; // 8 byte, 后5字节有用，高3字节未使用
    } Body_Unit_t;
    '''

    # 检查输入数据长度是否为8字节
    if len(data) != 8:
        raise ValueError("Data must be exactly 8 bytes long.")

    # 解析数据
    rawtime = data[:5]  # 前5字节
    address = struct.unpack('B', data[5:6])[0]  # 第6字节
    seqNumber = struct.unpack('H', data[6:8])[0]  # 最后2字节

    # 假设dwTime_t是一个8字节整数，我们仅使用后5字节
    # 由于高3字节未使用，我们需要正确处理字节顺序
    dwTime_t = struct.unpack('<Q', b'\x00\x00\x00' + data[3:8])[0]  # 使用小端字节序，并补充3个零字节到最高位
    dwTime_ms = (dwTime_t & 0xffffffffff)
    return {
        'address': address,
        'seqNumber': seqNumber,
        'dwTime_ms': dwTime_ms
    }

def parse_header(header_orginal_data):
    '''
    /* Ranging Message Header*/
      typedef struct {
      uint16_t srcAddress; // 2 byte
      uint16_t msgSequence; // 2 byte
      Timestamp_Tuple_t_2 lastTxTimestamps[RANGING_MAX_Tr_UNIT]; // 8 byte * MAX_Tr_UNIT default = 5 
      uint16_t msgLength; // 2 byte
      uint16_t filter; // 16 bits bloom filter
      float posiX;     // 4 byte
      float posiY;
      float posiZ;
    } __attribute__((packed)) Ranging_Message_Header_t; // 10 byte + 10 byte * MAX_Tr_UNIT

    typedef union {
      struct {
        uint8_t rawtime[5];   // 5 bytes
        uint8_t address;      // 1 byte
        uint16_t seqNumber;   // 2 bytes
      } __attribute__((packed));
      dwTime_t timestamp;     // 8 bytes (only the lower 5 bytes are meaningful)
    } Timestamp_Tuple_t_2;

    '''
    format_string = '<H H ' + 'Q' * 5 + ' H H f f f'
    # format_string = '<H H ' + '5s B H' * 5 + ' H H f f f'
    header_unpacked_data = struct.unpack(format_string, header_orginal_data)
    header = {
        'srcAddress': header_unpacked_data[0],
        'msgSequence': header_unpacked_data[1],
        'lastTxTimestamps':(header_unpacked_data[2]),
        # 'lastTxTimestamp1':{"timestamp":header_unpacked_data[2]& 0xffffffffff,'address':header_unpacked_data[3],'seqNumber':header_unpacked_data[4]},
        # 'lastTxTimestamp2':{"timestamp":header_unpacked_data[5]& 0xffffffffff,'address':header_unpacked_data[6],'seqNumber':header_unpacked_data[7]},
        # 'lastTxTimestamp3':{"timestamp":header_unpacked_data[8]& 0xffffffffff,'address':header_unpacked_data[9],'seqNumber':header_unpacked_data[10]},
        # 'lastTxTimestamp4':{"timestamp":header_unpacked_data[11]& 0xffffffffff,'address':header_unpacked_data[12],'seqNumber':header_unpacked_data[13]},
        # 'lastTxTimestamp5':{"timestamp":header_unpacked_data[14]& 0xffffffffff,'address':header_unpacked_data[15],'seqNumber':header_unpacked_data[16]},
        # 'posiX':header_unpacked_data[17],
        # 'posiY':header_unpacked_data[18],
        # 'posiZ':header_unpacked_data[19],
        
    }
    return header

def unpacked(x: pd.DataFrame):
    binData = x['bin_data']
    if(len(binData)<20):
        return x
    
    x['header'] = parse_header(binData[0:60])
    base = 52
    body_unit_size = 8
    len_bin_data = len(binData)
    i = 0
    while base + body_unit_size <= len_bin_data:
        i = i + 1
        body_unit = parse_body_unit(binData[base:base + body_unit_size])
        base += body_unit_size
        x['body_unit_' + str(i)] = body_unit
    return x


data.loc[:, 'sniffer_rx_time'] = data.loc[:, 'sniffer_rx_time'] & 0xffffffffff  # 得到正确的时间戳
# data.loc[:, 'sniffer_rx_time'] = data.loc[:, 'sniffer_rx_time'] * DW_TIME_TO_MS  # 换算成ms
data.loc[:, 'diff'] = data.loc[:, 'sniffer_rx_time'] - data.loc[:, 'sniffer_rx_time'].shift(1)
data.loc[:, 'diff'].fillna(0, inplace=True)
data = data.apply(process_overflow, axis=1)
data: pd.DataFrame
data.reset_index(inplace=True)
data.loc[:, 'sniffer_rx_time'] = data.loc[:, 'sniffer_rx_time'] - data.loc[0, 'sniffer_rx_time']

START_POINT = 0  # 起始数据点
STOP_POINT = len(data) - 1  # 最后数据点
start_rv_time = data.loc[START_POINT, 'sniffer_rx_time']
stop_rv_time = data.loc[STOP_POINT, 'sniffer_rx_time']
data = data.loc[START_POINT:STOP_POINT].apply(unpacked, axis=1)

drop_coloums = ['magic', 'bin_data','diff']
print(data.columns)
data = data.drop(drop_coloums,axis=1)
data.to_csv("swarm_data.csv")
