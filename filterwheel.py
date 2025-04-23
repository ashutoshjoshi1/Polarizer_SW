import time
import sys
import argparse
import serial
from serial.tools import list_ports


def find_pl2303_port():
    """
    Scan available serial ports for a Prolific PL2303GS device and return its port name.
    """
    for port in list_ports.comports():
        # Prolific PL2303GS vendor and product IDs
        if port.vid == 0x067B and port.pid == 0x2303:
            return port.device
    return None


class FilterWheelController:
    def __init__(self, port=None, baudrate=9600, timeout=1):
        # Auto-detect port if not provided
        if port is None:
            port = find_pl2303_port()
            if port is None:
                raise IOError("PL2303GS serial port not found. Please specify a port with --port.")
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)  # Allow time for the port to initialize

    def move(self, wheel: int, position: int) -> str:
        """
        Send a move command to the filter wheel.
        The command format is 'F{wheel}{position}', where wheel is the wheel number (1)
        and position is an integer 1-9.
        Returns the device's response (if any).
        """
        if wheel != 1:
            raise ValueError("Only filter wheel 1 is supported.")
        if not 1 <= position <= 9:
            raise ValueError("Position must be between 1 and 9.")

        cmd = f"F{wheel}{position}"
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()

        # Read line (if the device sends an acknowledgement)
        try:
            resp = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception:
            resp = ''
        return resp

    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control a filter wheel via PL2303GS USB-to-serial bridge.")
    parser.add_argument('-p', '--port', help='Serial port (e.g., COM3 or /dev/ttyUSB0). Auto-detected if omitted.')
    parser.add_argument('-b', '--baudrate', type=int, default=9600, help='Serial baud rate (default: 9600)')
    parser.add_argument('-w', '--wheel', type=int, default=1, choices=[1], help='Filter wheel number (only 1 supported)')
    parser.add_argument('-pos', '--position', type=int, required=True, choices=list(range(1,10)), help='Target position (1-9)')
    args = parser.parse_args()

    try:
        controller = FilterWheelController(port=args.port, baudrate=args.baudrate)
        response = controller.move(args.wheel, args.position)
        print(f"Sent command F{args.wheel}{args.position}. Device response: '{response}'")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if 'controller' in locals():
            controller.close()
