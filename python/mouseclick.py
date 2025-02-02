from pynput.mouse import Controller, Button
from serial_worker import SerialWorker

PORT = "COM4"
BAUDRATE = 115200

serial_worker = SerialWorker(port=PORT, baudrate=BAUDRATE)
analogRead = 0.0

mouse = Controller()
is_pressed = False  # Track mouse button state

def read_data(serial_worker):
    global analogRead
    analogRead = 0.0
    sensor_data = serial_worker.get_sensor_data()

    if sensor_data and 'analogRead' in sensor_data:
        analogRead = sensor_data['analogRead']

def main():
    global is_pressed
    serial_worker.connect_to_port()

    while True:
        read_data(serial_worker)

        # If pressure is high, press mouse button (hold)
        if analogRead > 100 and not is_pressed:  # Adjust threshold as needed
            mouse.press(Button.left)
            is_pressed = True
            print(f"Mouse Pressed (Analog Value: {analogRead})")

        # If pressure is low, release mouse button
        elif analogRead <= 100 and is_pressed:
            mouse.release(Button.left)
            is_pressed = False
            print(f"Mouse Released (Analog Value: {analogRead})")

if __name__ == '__main__':
    main()
