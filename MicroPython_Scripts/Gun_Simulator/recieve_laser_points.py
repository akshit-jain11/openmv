import serial
import struct

PORT       = "/dev/ttyACM1"
BAUD       = 115200
BLOB_MAGIC = b'\xAB\xCD\xEF\x01'
BLOB_PAYLOAD_SIZE = 8   # 2 (cx) + 2 (cy) + 4 (roundness)

def recv_exact(ser, n):
    buf = b""
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise IOError("Serial timeout / disconnected")
        buf += chunk
    return buf

def sync_to_magic(ser):
    buf = b""
    while True:
        buf += ser.read(1)
        if len(buf) > 4:
            buf = buf[-4:]
        if buf == BLOB_MAGIC:
            return True

ser = serial.Serial(PORT, baudrate=BAUD, timeout=5)
print(f"Listening on {PORT} ...")

while True:
    sync_to_magic(ser)

    # Read 4-byte length
    size = int.from_bytes(recv_exact(ser, 4), 'little')

    if size != BLOB_PAYLOAD_SIZE:
        print(f"Unexpected payload size: {size}, resyncing...")
        continue

    # Read and unpack cx, cy, roundness
    payload = recv_exact(ser, size)
    cx, cy, roundness = struct.unpack('<HHf', payload)

    print(f"cx={cx:3d}  cy={cy:3d}  roundness={roundness:.3f}")
