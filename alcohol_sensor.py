import json
import sys

import serial


with serial.Serial("/dev/ttyUSB0", 115200) as s:
    buffer = b''
    while b'\r' not in buffer:
        buffer = s.readline()
        if not buffer:
            print("Error reading data from serial port.", file=sys.stderr)
            sys.exit(0)
    buffer = b''
    while True:
        while b'\r' not in buffer:
            data = s.readline()
            if not data:
                print("Error reading data from serial port.", file=sys.stderr)
                sys.exit(0)
            buffer += data
        data_list = buffer.split(b'\r')
        # print(data_list)
        last_one = data_list[-1]
        if last_one == b'\n' or last_one == b'':
            buffer = b''
        else:
            buffer = last_one
        if len(data_list) > 1:
            try:
                result = json.loads(data_list[-2])
            except:
                print("Bad format.")
                continue
            print(result)
