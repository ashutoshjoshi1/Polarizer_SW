import serial
import time

SERIAL_PORT = 'COM16'
BAUD_RATE = 19200
TIMEOUT = 2

def send_command(ser, command_hex):
    command_bytes = bytes.fromhex(command_hex)
    ser.reset_input_buffer()  # Clear old data
    ser.write(command_bytes)
    print(f"Sent: {command_hex}")
    time.sleep(0.3)
    response = ser.read_all()
    print(f"Raw response: {response}")
    return response

def read_temperature(ser):
    response = send_command(ser, '4C')
    if len(response) < 2:
        print("Invalid response length.")
        return None
    temp_raw = int.from_bytes(response[:2], byteorder='big')
    temperature = temp_raw / 100.0
    return temperature

def set_temperature(ser, target_temp):
    temp_value = int(target_temp * 100)
    temp_bytes = temp_value.to_bytes(2, byteorder='big')
    command = '1C' + temp_bytes.hex()
    send_command(ser, command)
    print(f"Target temperature set to {target_temp:.1f}°.")

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print("Connected to device.")
            current_temp = read_temperature(ser)
            if current_temp is not None:
                print(f"Current Temperature: {current_temp}°C")
            else:
                print("Failed to read temperature.")

            set_temperature(ser, 25.0)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == '__main__':
    main()
