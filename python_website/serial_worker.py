import serial
import time


class Serial_Worker():
    def __init__(self, port, baudrate) -> None:
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.retry_seconds = 5
        self.bytesize = 8
        self.timeout = 2
    
    
    def connect_to_port(self):
        if self.ser and self.ser.is_open:
            return
        
        try:
            print(f"Trying to connect to {self.port}...")
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=self.bytesize, timeout=self.timeout)
            print(f"Connected to {self.port}")

        except serial.SerialException as e:
            print(f"Failed to connect: {e}. Retrying in {self.retry_seconds} seconds...")
            self.ser = None
            time.sleep(self.retry_seconds)

    def decode_data(self, raw_data):
        try:
            return raw_data.decode("utf-8").strip()
        except UnicodeDecodeError:
            print(f"Error decoding data: {raw_data}")
            return None
        
    def is_ascii_printable(self,data):
        return all(32 <= b <= 126 or b in (10, 13) for b in data)

    def handle_serial_data(self,raw_data):
        line = raw_data.decode('utf-8').strip()
        if line is not None and line.startswith("Gyro:"):
            return line
    
    def get_gyro_values(self):       
        if not self.ser or not self.ser.is_open:
            return None
        
        try: 
            raw_data = self.ser.readline()
            if self.is_ascii_printable(raw_data):
                data = self.decode_data(raw_data)
                if data and data.startswith("Gyro:"):
                    try:
                        _, x, y, z = data.split()  # Example: "Gyro: -10 15 -20"
                        x = int(x.strip(','))
                        y = int(y.strip(','))
                        z = int(z.strip(','))
                        return [x, y, z]
                    except ValueError as e:
                        print(f"Error parsing gyro data: {data}. Error: {e}")
                        return None
        except serial.SerialException as e:
            print(f"Serial error: {e}. Closing connection...")
            self.ser.close()
            self.ser = None
        except Exception as e:
            print(f"Unexpected error: {e}")
        return None 