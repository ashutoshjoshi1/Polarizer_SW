import serial
import time

# Replace with your actual COM port
SERIAL_PORT = 'COM16'
BAUD_RATE = 19200  # Adjust if necessary
TIMEOUT = 2  # seconds

def send_command(ser, command_hex):
    """
    Sends a command to the controller and returns the response.
    """
    command_bytes = bytes.fromhex(command_hex)
    ser.write(command_bytes)
    time.sleep(0.1)  # Wait for the device to process
    response = ser.read(2)  # Expecting a 2-byte response
    return response

def read_temperature(ser):
    """
    Reads the current temperature from the controller.
    """
    response = send_command(ser, '4C')  # Command to read current temperature
    if len(response) != 2:
        print("Invalid response length.")
        return None
    temp_raw = int.from_bytes(response, byteorder='big')
    temperature = temp_raw / 100.0  # Convert to °C or °F based on device setting
    return temperature

def set_temperature(ser, target_temp):
    """
    Sets the desired control temperature on the controller.
    """
    temp_value = int(target_temp * 100)
    temp_hex = temp_value.to_bytes(2, byteorder='big').hex()
    command = '1C' + temp_hex  # Command to set desired control temperature
    send_command(ser, command)

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            current_temp = read_temperature(ser)
            if current_temp is not None:
                print(f"Current Temperature: {current_temp}°")
            else:
                print("Failed to read temperature.")

            # Example: Set target temperature to 25.0°
            set_temperature(ser, 25.0)
            print("Target temperature set to 25.0°.")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")

if __name__ == '__main__':
    main()
