import math

try:
    import libscrc
    def modbus_crc16(data: bytes) -> int:
        """Calculate Modbus CRC-16 for the given data bytes."""
        return libscrc.modbus(data)
except ImportError:
    def modbus_crc16(data: bytes) -> int:
        """Calculate Modbus CRC-16 for the given data bytes (fallback implementation)."""
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

def compute_sun_vector(lat: float, lon: float):
    """Compute the sun's direction vector given latitude & longitude."""
    from astral import Observer
    from astral.sun import azimuth, elevation
    from datetime import datetime, timezone
    obs = Observer(latitude=lat, longitude=lon)
    now = datetime.now(timezone.utc)
    try:
        az = azimuth(obs, now)
        el = elevation(obs, now)
    except Exception:
        # If Astral fails (e.g., sun below horizon), default to sun at horizon (south)
        az = 180.0
        el = 0.0
    # Convert to radians
    az_rad = math.radians(az)
    el_rad = math.radians(el)
    # Convert spherical (azimuth, elevation) to Cartesian unit vector
    # Astral: azimuth 0° = north, 90° = east; align device coords: X east, Y north, Z up
    sx = math.cos(el_rad) * math.sin(az_rad)
    sy = math.cos(el_rad) * math.cos(az_rad)
    sz = math.sin(el_rad)
    # Scale the vector for plotting (position the sun point away from origin)
    return (sx * 3.0, sy * 3.0, sz * 3.0)
