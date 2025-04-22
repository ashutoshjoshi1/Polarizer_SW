import serial
import time

MONITOR_PORT = 'COM16'
BAUD_RATE = 19200       # Adjust to match original software
TIMEOUT = 2

def main():
    try:
        with serial.Serial(
            MONITOR_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        ) as ser:
            print(f"Listening on {MONITOR_PORT} at {BAUD_RATE} baud...\n")

            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    hex_dump = " ".join(f"{byte:02X}" for byte in data)
                    ascii_dump = "".join(chr(b) if 32 <= b <= 126 else '.' for b in data)
                    print(f"HEX: {hex_dump}   | ASCII: {ascii_dump}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    main()
