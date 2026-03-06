import serial
import time
import struct

# Configuration
SERIAL_PORT = '/dev/ttyS0'  # Use /dev/ttyAMA0 or /dev/ttyS0 depending on Pi config
BAUD_RATE = 921600            # Matches your Arduino code
# BAUD_RATE = 115200            # Matches your Arduino code

def calculate_distance(byte_8, byte_9, byte_10):
    # Replicates the Arduino logic: ((long)(rx[10]<<24 | rx[9]<<16 | rx[8]<<8) / 256) / 1000.0
    # This is essentially a 24-bit value shifted to the correct scale
    val = (byte_10 << 16) | (byte_9 << 8) | byte_8
    # Handle signed 24-bit if necessary, though usually distance is positive
    if val > 0x7FFFFF:
        val -= 0x1000000
    return val / 1000.0

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE}")
except Exception as e:
    print(f"Failed to open serial port: {e}")
    exit()

def read_tof_data():
    while True:
        # 1. Look for the frame header 0x57 0x00
        header = ser.read(1)
        if not header or header[0] != 0x57:
            continue
        
        header2 = ser.read(1)
        if not header2 or header2[0] != 0x00:
            continue

        # 2. Read the remaining 14 bytes (total packet is 16 bytes)
        payload = ser.read(14)
        if len(payload) < 14:
            continue

        # Create full buffer for checksum calculation
        full_packet = bytes([0x57, 0x00]) + payload
        
        # 3. Verify Checksum (Sum of first 15 bytes == 16th byte)
        check_sum = sum(full_packet[:15]) & 0xFF
        if check_sum != full_packet[15]:
            # print("Checksum failed")
            continue

        # 4. Parse Data
        # Mapping based on your Arduino structure:
        # rx_buf[3] = ID
        # rx_buf[4-7] = System Time (Little Endian)
        # rx_buf[8-10] = Distance (3 bytes used for the 24-bit calculation)
        # rx_buf[11] = Status
        # rx_buf[12-13] = Signal Strength
        # rx_buf[14] = Precision
        
        id_val = full_packet[3]
        
        # System Time (4 bytes, Little Endian)
        sys_time = struct.unpack('<I', full_packet[4:8])[0]
        
        # Distance (matching your Arduino math)
        dist = calculate_distance(full_packet[8], full_packet[9], full_packet[10])
        
        status = full_packet[11]
        strength = struct.unpack('<H', full_packet[12:14])[0]
        precision = full_packet[14]

        return {
            "id": id_val,
            "system_time": sys_time,
            "dis": dist,
            "dis_status": status,
            "signal_strength": strength,
            "range_precision": precision
        }

if __name__=="__main__":
    try:
        while True:
            data = read_tof_data()
            if data:
                print(f"ID: {data['id']} | Dist: {data['dis']:.3f}m | Strength: {data['signal_strength']} | Time: {data['system_time']}ms")
            
            # Adjust sleep as needed; your Arduino code had delay(1000)
            time.sleep(1/50)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close() 
