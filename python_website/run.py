from flask import Flask, render_template
from flask_socketio import SocketIO
from serial_worker import Serial_Worker
from eventlet import monkey_patch
import threading

app = Flask(__name__)
socketio = SocketIO(app,async_mode='threading')

@app.route('/')
def index():
    return render_template('index.html')

PORT = "COM4"
BAUDRATE = 115200

serial_worker = Serial_Worker(port=PORT, baudrate=BAUDRATE)
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
            gyro_values = serial_worker.get_gyro_values()
            if gyro_values:
                print(f"Gyro: {gyro_values}")
                socketio.emit('gyro_data', {
                    'x': gyro_values[0],
                    'y': gyro_values[1],
                    'z': gyro_values[2]
                })
        socketio.sleep(0.1) 

if __name__ == '__main__':
    socketio.run(app, port=5050, debug=True)
