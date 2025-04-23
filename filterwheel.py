import serial, time
from PyQt5.QtCore import QThread, pyqtSignal

class FilterWheelCommandThread(QThread):
    """Background thread to send a command to the filter wheel and read its position."""
    result_signal = pyqtSignal(object, str)  # emits (position, status_message)
    def __init__(self, serial_obj, command, parent=None):
        super().__init__(parent)
        self.serial = serial_obj       # an open serial.Serial instance
        self.command = command.strip() # e.g. "F1r", "F15", "F19" or "?"
    def run(self):
        try:
            # Ensure input buffer is clear before sending
            self.serial.reset_input_buffer()
            # Send the command with CR termination
            cmd_str = self.command + "\r"
            self.serial.write(cmd_str.encode('utf-8'))
            # If it's a move/reset command, wait and then query position
            if self.command != "?":
                time.sleep(1.0)  # wait for the wheel to move/reset
                self.serial.reset_input_buffer()      # flush any interim response
                self.serial.write(b"?\r")             # query current position
            # Read the response line (with timeout)
            response = self.serial.readline()  # reads until '\n' or timeout
            pos = None
            if response:
                data = response.decode('ascii', errors='ignore').strip()
                # Parse any digit(s) in the response as the position
                pos = int(''.join(filter(str.isdigit, data))) if any(c.isdigit() for c in data) else None
                # Determine a user-friendly status message
                if pos is not None:
                    if self.command.endswith('r'):  # reset command
                        msg = f"Filter wheel reset to position {pos}."
                    elif self.command == "?":       # query command
                        msg = f"Filter wheel is at position {pos}."
                    else:                           # move command like F15, F19
                        msg = f"Filter wheel moved to position {pos}."
                else:
                    # Received a response that didn't contain a position
                    msg = f"Received: {data}"
            else:
                # No response (e.g. timeout)
                msg = "No response from filter wheel (timeout)."
            # (Do not close the port here to keep connection alive)
        except Exception as e:
            pos = None
            msg = f"Serial error: {e}"
            try:
                self.serial.close()  # ensure port is closed on error
            except: 
                pass
        # Emit the result (position may be None if failed or unknown)
        self.result_signal.emit(pos, msg)

class FilterWheelConnectThread(QThread):
    """Background thread to open the filter wheel serial port without blocking UI."""
    result_signal = pyqtSignal(object, str)  # emits (serial_obj, status_message)
    def __init__(self, port_name, parent=None):
        super().__init__(parent)
        self.port = port_name
    def run(self):
        try:
            ser = serial.Serial(self.port, baudrate=4800, timeout=1)
            msg = f"Filter wheel connected on {self.port}"
        except Exception as e:
            ser = None
            msg = f"Failed to open {self.port}: {e}"
        self.result_signal.emit(ser, msg)
