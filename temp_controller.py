import serial
import time

SERIAL_PORT = 'COM16'
BAUD_RATE = 9600 
TIMEOUT = 2

def read_temperature(ser):
    ser.write(b'R\r\n')
    time.sleep(0.2)
    response = ser.readline().decode().strip()
    print(f"Current Temperature: {response}")
    return response

def set_temperature(ser, target_temp):
    command = f'S{target_temp:.1f}\r\n'
    ser.write(command.encode())
    print(f"Set temperature command sent: {command.strip()}")

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            current = read_temperature(ser)
            set_temperature(ser, 20.0)
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")

if __name__ == '__main__':
    main()

