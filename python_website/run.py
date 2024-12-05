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

    while not stop_event.is_set(): 
        if serial_worker.ser and serial_worker.ser.is_open:
            sensor_data = serial_worker.get_sensor_data()
            if sensor_data:
                yaw = sensor_data.get("yaw")
                pitch = sensor_data.get("pitch")
                roll = sensor_data.get("roll")
                if yaw and pitch and roll:
                    socketio.emit('position_data', {
                            'x': yaw,
                            'y': pitch,
                            'z': roll
                         })


if __name__ == '__main__':
    socketio.run(app, port=5050, debug=True)
