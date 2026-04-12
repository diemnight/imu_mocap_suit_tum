import serial
import struct
import csv
import os
from datetime import datetime

PORT = '/dev/ttyACM0'
BAUD = 921600
HEADER_1 = 0xAA
HEADER_2 = 0xFF
PACKET_SIZE = 34  # 2 header + 4 ts + 16 quaternion + 12 acceleration

script_dir = os.path.dirname(os.path.abspath(__file__))
save_dir = os.path.join(script_dir, 'imu_recordings')
os.makedirs(save_dir, exist_ok=True)

filename = os.path.join(save_dir, f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

ser = serial.Serial(PORT, BAUD, timeout=1)

outfile = open(filename, 'w', newline='')
writer = csv.writer(outfile)
writer.writerow(['timestamp_us', 'qw', 'qi', 'qj', 'qk', 'ax', 'ay', 'az'])

print(f"Recording to {filename}")
print("Ctrl+C to stop\n")

def find_header(ser):
    while True:
        b1 = ser.read(1)
        if not b1 or b1[0] != HEADER_1:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != HEADER_2:
            continue
        return True

def is_valid(qw, qi, qj, qk):
    import math
    for v in [qw, qi, qj, qk]:
        if math.isnan(v) or math.isinf(v) or abs(v) > 1.1:
            return False
    mag = math.sqrt(qw**2 + qi**2 + qj**2 + qk**2)
    return 0.9 < mag < 1.1

try:
    find_header(ser)
    while True:
        raw = ser.read(PACKET_SIZE - 2)  # header already consumed
        if len(raw) < PACKET_SIZE - 2:
            find_header(ser)
            continue

        ts, qw, qi, qj, qk, ax, ay, az = struct.unpack('<Ifffffff', raw)

        if not is_valid(qw, qi, qj, qk):
            find_header(ser)
            continue

        writer.writerow([ts, qw, qi, qj, qk, ax, ay, az])
        outfile.flush()
        print(f"t={ts:10d}  w={qw:+.4f}  i={qi:+.4f}  j={qj:+.4f}  k={qk:+.4f}"
              f"  ax={ax:+.3f}  ay={ay:+.3f}  az={az:+.3f}")

        find_header(ser)

except KeyboardInterrupt:
    print(f"\nSaved to {filename}")
    outfile.close()
    ser.close()