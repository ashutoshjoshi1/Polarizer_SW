import serial
import time
from serial.tools import list_ports

# Command Variants
def get_command_variants():
    plain = bytes.fromhex('4C')
    stx_etx = b'\x02' + plain + b'\x03'  # <STX> 4C <ETX>
    with_checksum = stx_etx + bytes([sum(stx_etx) & 0xFF])  # Add simple checksum

    return {
        'plain_4C': plain,
        'stx_etx': stx_etx,
        'with_checksum': with_checksum
    }

# Serial config combinations
BAUD_RATES = [9600, 19200]
PARITY_OPTIONS = {
    'NONE': serial.PARITY_NONE,
    'ODD': serial.PARITY_ODD,
    'EVEN': serial.PARITY_EVEN
}

PORT = 'COM16'  # Update if needed
TIMEOUT = 1

def try_config(port, baud, parity, command_bytes, label):
    try:
        with serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=parity,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        ) as ser:
            ser.reset_input_buffer()
            print(f"\n[TEST] Baud: {baud}, Parity: {parity}, Cmd: {label}")
            ser.write(command_bytes)
            time.sleep(0.3)
            response = ser.read_all()
            print(f"Response: {response}")
    except Exception as e:
        print(f"Error: {e}")

def main():
    commands = get_command_variants()

    for baud in BAUD_RATES:
        for parity_label, parity_val in PARITY_OPTIONS.items():
            for cmd_label, cmd_bytes in commands.items():
                try_config(PORT, baud, parity_val, cmd_bytes, cmd_label)

if __name__ == '__main__':
    main()
