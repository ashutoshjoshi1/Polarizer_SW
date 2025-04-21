import struct
import threading

def parse_imu_packet(packet: bytes):
    """Parse an 11-byte WitMotion IMU packet. Returns a tuple (label, *values)."""
    # Packet structure: [0]=0x55, [1]=ID, [2:10]=data, [10]=checksum
    data_bytes = packet[2:10]  # 8 data bytes (little-endian)
    packet_id = packet[1]
    if packet_id == 0x53:
        # Euler angles (roll, pitch, yaw) + temperature (not used here)
        roll_raw, pitch_raw, yaw_raw, temp_raw = struct.unpack('<hhhH', data_bytes)
        roll = roll_raw / 32768.0 * 180.0
        pitch = pitch_raw / 32768.0 * 180.0
        yaw = yaw_raw / 32768.0 * 180.0
        return ("Angle", roll, pitch, yaw)
    elif packet_id == 0x56:
        # Pressure & altitude (treat altitude as temperature if provided)
        pressure_raw, alt_raw, _, _ = struct.unpack('<hhHH', data_bytes)
        pressure = pressure_raw / 100.0    # hPa
        temperature = alt_raw / 100.0      # °C (from pressure packet, often altitude field)
        return ("Pressure", pressure, temperature)
    elif packet_id == 0x57:
        # GPS latitude and longitude (int32 each, scaled by 1e-7)
        if data_bytes == b'\x00\x00\x00\x00\x00\x00\x00\x00':
            return ("GPS", None, None)
        try:
            lon_raw, lat_raw = struct.unpack('<ii', data_bytes)
            lat = lat_raw / 1e7
            lon = lon_raw / 1e7
            return ("GPS", lat, lon)
        except Exception:
            return ("GPS", None, None)
    elif packet_id == 0x51:
        # Acceleration (Ax, Ay, Az) + internal temperature
        ax_raw, ay_raw, az_raw, temp_raw = struct.unpack('<hhhH', data_bytes)
        ax = ax_raw / 32768.0 * 16.0   # g (±16g range)
        ay = ay_raw / 32768.0 * 16.0   # g
        az = az_raw / 32768.0 * 16.0   # g
        return ("Accel", ax, ay, az)
    elif packet_id == 0x52:
        # Angular velocity (Gx, Gy, Gz) + internal temperature
        gx_raw, gy_raw, gz_raw, temp_raw = struct.unpack('<hhhH', data_bytes)
        gx = gx_raw / 32768.0 * 2000.0  # °/s (±2000 °/s range)
        gy = gy_raw / 32768.0 * 2000.0  # °/s
        gz = gz_raw / 32768.0 * 2000.0  # °/s
        return ("Gyro", gx, gy, gz)
    elif packet_id == 0x54:
        # Magnetometer (Hx, Hy, Hz) + internal temperature
        mx_raw, my_raw, mz_raw, temp_raw = struct.unpack('<hhhH', data_bytes)
        # Assume ±1000 µT range for magnetometer output
        mx = mx_raw / 32768.0 * 1000.0  # µT
        my = my_raw / 32768.0 * 1000.0  # µT
        mz = mz_raw / 32768.0 * 1000.0  # µT
        return ("Mag", mx, my, mz)
    else:
        return ("Unknown", None)

def read_from_imu(serial_obj, data_dict: dict, stop_event: threading.Event):
    """Continuously read and parse data from the IMU serial port until stop_event is set or serial is closed."""
    buffer = []
    # Loop while serial is open and stop event not set
    while serial_obj.is_open and not stop_event.is_set():
        byte = serial_obj.read()
        if not byte:
            continue
        buffer.append(byte[0] if isinstance(byte, (bytes, bytearray)) else ord(byte))
        # Process packets of 11 bytes
        if len(buffer) >= 11:
            # Check packet start and checksum
            if buffer[0] == 0x55 and (sum(buffer[:10]) & 0xFF) == buffer[10]:
                packet_bytes = bytes(buffer[:11])
                buffer = buffer[11:]  # remove processed packet
                label, *values = parse_imu_packet(packet_bytes)
                # Update shared data dictionary with latest readings
                if label == "Angle":
                    roll, pitch, yaw = values
                    data_dict["rpy"] = (roll, pitch, yaw)
                elif label == "Pressure":
                    pressure, temp = values
                    data_dict["pressure"] = pressure
                    data_dict["temperature"] = temp
                elif label == "GPS":
                    lat, lon = values
                    if lat is not None and lon is not None:
                        data_dict["latitude"] = lat
                        data_dict["longitude"] = lon
                elif label == "Accel":
                    ax, ay, az = values
                    data_dict["accel"] = (ax, ay, az)
                elif label == "Gyro":
                    gx, gy, gz = values
                    data_dict["gyro"] = (gx, gy, gz)
                elif label == "Mag":
                    mx, my, mz = values
                    data_dict["mag"] = (mx, my, mz)
                # (Ignore "Unknown" packets or other IDs not handled)
            else:
                # Invalid packet start/checksum, discard one byte and resync
                buffer.pop(0)

def start_imu_read_thread(serial_obj, data_dict: dict):
    """Start a background thread to continuously read from IMU and update data_dict. Returns a stop_event to signal termination."""
    stop_event = threading.Event()
    thread = threading.Thread(target=read_from_imu, args=(serial_obj, data_dict, stop_event), daemon=True)
    thread.start()
    return stop_event
