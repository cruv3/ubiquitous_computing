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

def serial_start_listening():
    serial_worker.connect_to_port()
    initialization_data = []
    is_initialized = False 
    data_buffer = []

    while not stop_event.is_set(): 
        if serial_worker.ser and serial_worker.ser.is_open:
            sensor_data = serial_worker.get_sensor_data()
            if sensor_data:
                accel = sensor_data.get("accel")
                gyro = sensor_data.get("gyro")
                mag = sensor_data.get("mag")
                if accel and gyro and mag:
                    imu_data = np.hstack((gyro, accel, mag)).reshape(1, -1)
                    if not is_initialized:
                        initialization_data.append(imu_data)
                        if len(initialization_data) >= 100:  # Wenn 100 Daten gesammelt wurden
                            initialization_data = np.vstack(initialization_data)
                            processor.initialize(initialization_data)
                            is_initialized = True
                            print("IMU initialized!")
                    elif is_initialized:
                        data_buffer.append(imu_data)
                        # Process data once the buffer has sufficient data
                        if len(data_buffer) >= 1:  # Adjust buffer size as needed
                            data_to_process = np.vstack(data_buffer)
                            position = processor.process({
                                'gyro': data_to_process[:, :3],
                                'accel': data_to_process[:, 3:6],
                                'mag': data_to_process[:, 6:]
                            })
                            print(position)
                            data_buffer.clear()  # Clear buffer after processing

                            # Emit position data
                            socketio.emit('position_data', {
                                'x': position[0],
                                'y': position[1],
                                'z': position[2]
                            })


if __name__ == '__main__':
    socketio.run(app, port=5050, debug=True)
