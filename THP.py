import serial
import json
import time

def read_sensor_data(port_name, baud_rate=9600, timeout=1):
    """
    Connect to hardware via serial port, send the 'p' command to get sensor data,
    and parse the JSON response to extract temperature, humidity, and pressure.
    
    Args:
        port_name (str): Serial port name (e.g. 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
        baud_rate (int): Baud rate for serial communication
        timeout (int): Serial timeout in seconds
        
    Returns:
        dict: Dictionary containing temperature, humidity, and pressure values
    """
    try:
        # Open serial connection
        ser = serial.Serial(port_name, baud_rate, timeout=timeout)
        print(f"Connected to {port_name}")
        
        # Allow time for connection to establish
        time.sleep(1)
        
        # Send 'p' command to get sensor data (as bytes)
        ser.write(b'p\r\n')
        
        # Read response (assuming the response ends with a newline character)
        response = ""
        start_time = time.time()
        
        # Read data with timeout
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                response += line
                
                # Try to parse as JSON to see if we have a complete response
                try:
                    data = json.loads(response)
                    # If parsing succeeds, break out of the loop
                    break
                except json.JSONDecodeError:
                    # Not a complete JSON yet, continue reading
                    continue
        
        # Close the serial connection
        ser.close()
        
        # Parse the JSON response
        try:
            data = json.loads(response)
            
            # Extract temperature, humidity, and pressure
            # (Adjust the keys based on the actual JSON structure from your hardware)
            temperature = data.get('Temperature')
            humidity = data.get('Humidity')
            pressure = data.get('Pressure')
            
            return {
                'temperature': temperature,
                'humidity': humidity,
                'pressure': pressure
            }
            
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
            print(f"Received: {response}")
            return None
            
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        return None

def main():
    # Replace 'COM3' with your actual serial port
    # Common ports:
    # - Windows: 'COM1', 'COM2', 'COM3', etc.
    # - Linux: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    # - Mac: '/dev/tty.usbserial', '/dev/tty.usbmodem', etc.
    PORT_NAME = 'COM3'  # Update this to match your hardware's port
    
    # You can also adjust the baud rate if needed
    BAUD_RATE = 9600
    
    sensor_data = read_sensor_data(PORT_NAME, BAUD_RATE)
    
    if sensor_data:
        print("\nSensor readings:")
        print(f"Temperature: {sensor_data['temperature']}")
        print(f"Humidity: {sensor_data['humidity']}")
        print(f"Pressure: {sensor_data['pressure']}")
    else:
        print("Failed to retrieve sensor data.")

if __name__ == "__main__":
    main()
