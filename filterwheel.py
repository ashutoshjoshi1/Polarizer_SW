import serial
import time

def rotate_filterwheel1():
    PORT = 'COM17'
    BAUD = 4800
    TIMEOUT = 1.0

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT,
        xonxoff=False,
        rtscts=False
        )

    try:
        ser.write("?\r".encode('utf-8'))
        print("Sent → F1r")

        time.sleep(1)
        resp = ser.readline()
        if resp:
            print("Recv ←", resp.decode('ascii', 'replace').strip())
        else:
            print("No response (timeout)")

    finally:
        ser.close()
        print("Port closed.")

if __name__ == "__main__":
    rotate_filterwheel1()
