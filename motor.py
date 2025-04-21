import serial
from PyQt5.QtCore import QThread, pyqtSignal
import utils

# Motor control constants for Oriental Motor AZ series (Modbus)
TrackerSpeed = 10000       # Motor rotation speed (steps/s)
TrackerCurrent = 1000      # Motor current limit (in 0.1% units, 1000 = 100.0%)
SlaveID = 2                # Modbus slave address of the motor controller
BaudRateList = [9600, 19200, 38400, 57600, 115200, 230400]

class MotorConnectThread(QThread):
    """Thread to attempt motor serial connection with baud auto-detection."""
    result_signal = pyqtSignal(object, int, str)  # will emit (serial_obj or None, baud_rate, message)
    def __init__(self, port_name, parent=None):
        super().__init__(parent)
        self.port_name = port_name
    def run(self):
        found_serial = None
        found_baud = None
        message = ""
        # Try each baud rate to find a responding motor
        for baud in BaudRateList:
            try:
                ser = serial.Serial(
                    self.port_name, baudrate=baud, bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
                    timeout=0.5
                )
                # Example Modbus read command (function 0x03) at register 0x0058 (2 registers)
                base_cmd = bytes([SlaveID, 0x03, 0x00, 0x58, 0x00, 0x02])
                crc_val = utils.modbus_crc16(base_cmd)
                crc_bytes = crc_val.to_bytes(2, 'little')
                ser.write(base_cmd + crc_bytes)
                # Read a few bytes to detect any response
                response = ser.read(5)
                if response:
                    found_serial = ser
                    found_baud = baud
                    message = f"Motor connected on {self.port_name} at {baud} baud."
                    break
                ser.close()
            except Exception:
                # Ignore exceptions and try next baud
                continue
        if not found_serial:
            message = f"No response from motor on {self.port_name}."
        # Emit result (serial object if found, else None)
        self.result_signal.emit(found_serial, found_baud if found_baud else 0, message)

def send_move_command(serial_obj, angle: int) -> bool:
    """Send a move command to the motor to go to the specified angle (degrees). Returns True if ACK received."""
    # Construct Modbus Write Multiple Registers command to move motor to target angle
    base_cmd = bytes([SlaveID, 0x10, 0x00, 0x58, 0x00, 0x12, 0x24,
                      0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01])
    # Prepare data bytes for angle, speed, current
    try:
        angle_bytes = angle.to_bytes(4, 'big', signed=True)
    except OverflowError:
        # Clamp angle to 32-bit signed range if out of bounds
        val = max(min(angle, 0x7FFFFFFF), -0x80000000)
        angle_bytes = val.to_bytes(4, 'big', signed=True)
    speed_bytes = TrackerSpeed.to_bytes(4, 'big', signed=True)
    current_bytes = TrackerCurrent.to_bytes(4, 'big', signed=True)
    # Mid and end bytes (acceleration/deceleration and execution parameters)
    mid_bytes = bytes([0x00, 0x00, 0x1F, 0x40, 0x00, 0x00, 0x1F, 0x40])
    end_bytes = bytes([0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01])
    full_cmd = base_cmd + angle_bytes + speed_bytes + mid_bytes + current_bytes + end_bytes
    # Append CRC16
    crc_val = utils.modbus_crc16(full_cmd)
    crc_bytes = crc_val.to_bytes(2, 'little')
    try:
        serial_obj.reset_input_buffer()
        serial_obj.write(full_cmd + crc_bytes)
        # Response for function 0x10 (Write Multiple Registers) should be 8 bytes (including CRC)
        response = serial_obj.read(8)
        if response and len(response) >= 6 and response[1] == 0x10:
            return True
        else:
            return False
    except Exception:
        return False
