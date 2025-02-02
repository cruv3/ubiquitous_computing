from flask import Flask, render_template
from flask_socketio import SocketIO
from serial_worker import SerialWorker
from imu_processor import IMUProcessor
import numpy as np
import threading

app = Flask(__name__)
socketio = SocketIO(app,async_mode='threading')

@app.route('/')
def index():
    return render_template('index.html')

PORT = "COM4"
BAUDRATE = 115200

processor = IMUProcessor(sampling=100)
serial_worker = SerialWorker(port=PORT, baudrate=BAUDRATE)
stop_event = threading.Event() 

@socketio.on('connect')
def connect():
    stop_event.clear()
    threading.Thread(target=serial_start_listening, daemon=True).start()

@socketio.on('disconnect')
def disconnet():
    stop_event.set()

def collect_initial_data(sensor_data, num_samples=100):
    """Collects initial data samples from the IMU for initialization."""
    samples = []
    while len(samples) < num_samples:
        if sensor_data:
            sample = np.array(sensor_data['gyro'] + sensor_data['accel'] + sensor_data['mag'])
            samples.append(sample)
    return np.array(samples)

def serial_start_listening():
    serial_worker.connect_to_port()

    while not stop_event.is_set(): 
        if serial_worker.ser and serial_worker.ser.is_open:
            sensor_data = serial_worker.get_sensor_data()
            print(sensor_data)
            if sensor_data and all(key in sensor_data for key in ['gyro', 'accel', 'mag']):
                
                initial_data = collect_initial_data(sensor_data, num_samples=100)
                processor.initialize(initial_data)
                if sensor_data:
                    imu_data = {
                        'gyro': sensor_data['gyro'],
                        'accel': sensor_data['accel'],
                        'mag': sensor_data['mag']
                    }

                    # Process data to get position
                    position = processor.process(imu_data)
                    
                    # Emit position data
                    socketio.emit('position_data', {
                        'x': position[0],
                        'y': position[1],
                        'z': position[2]
                    })



if __name__ == '__main__':
    socketio.run(app, port=5050, debug=True)
