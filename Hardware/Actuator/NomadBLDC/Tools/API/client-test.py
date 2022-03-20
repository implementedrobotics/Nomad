import socketio

sio = socketio.Client()


@sio.event
def message(data):
    print('I received a message!')


@sio.event
def connect():
    print("Connected!")


@sio.event
def connect_error(data):
    print("The connection failed!")


@sio.on('hey')
def on_message(data):
    print('I received a message!')


sio.connect('http://localhost:8000/', socketio_path="ws/socket.io")
