import serial
import json
import time


class SerialWorker:
    def __init__(self, port, baudrate=115200, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect_to_port(self):
        if self.ser and self.ser.is_open:
            return

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"Connected with {self.port}")
        except serial.SerialException as e:
            print(f"Error while connecting: {e}")
            self.ser = None
            time.sleep(5)  # Erneuter Verbindungsversuch

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Closed connection.")

    def get_sensor_data(self):
        if not self.ser or not self.ser.is_open:
            return None     
        try: 
            raw_data = self.ser.readline().decode('utf-8').strip()
            time.sleep(0.01)
            if raw_data.startswith("{") and raw_data.endswith("}"):
                try:
                    data = json.loads(raw_data)
                    return data  # JSON-Daten zurückgeben
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {raw_data}, Error: {e}")
                    return None
        except Exception as e:
            print(f"Serial error: {e}")
            return None

