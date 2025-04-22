import serial
import time

SERIAL_PORT = 'COM16'
BAUD_RATE = 19200
TIMEOUT = 2

def send_command(ser, command_hex):
    """
    Sends a command to the controller and returns the response.
    """
    command_bytes = bytes.fromhex(command_hex)
    ser.write(command_bytes)
    time.sleep(0.1)
    response = ser.read(2)
    return response

def read_temperature(ser):
    """
    Reads the current temperature from the controller.
    """
    response = send_command(ser, '4C')
    if len(response) != 2:
        print("Invalid response length.")
        return None
    temp_raw = int.from_bytes(response, byteorder='big')
    temperature = temp_raw / 100.0
    return temperature

def set_temperature(ser, target_temp):
    """
    Sets the desired control temperature on the controller.
    """
    temp_value = int(target_temp * 100)
    temp_hex = temp_value.to_bytes(2, byteorder='big').hex()
    command = '1C' + temp_hex 
    send_command(ser, command)

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            current_temp = read_temperature(ser)
            if current_temp is not None:
                print(f"Current Temperature: {current_temp}°")
            else:
                print("Failed to read temperature.")

            set_temperature(ser, 0.0)
            print("Target temperature set to 0.0°.")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")

if __name__ == '__main__':
    main()
