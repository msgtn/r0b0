# from socketIO_client import SocketIO, LoggingNamespace
import socketio
def on_aaa_response(args):
    print('on_aaa_response', args['data'])

# socketIO = SocketIO('localhost', 4000, LoggingNamespace)
socketIO = socketio.Client()
port=4000
socketIO.connect(f'http://localhost:{port}')
# socketIO.on('aaa_response', on_aaa_response)
# socketIO.emit('aaa')
# socketIO.wait()
while 1:
    socketIO.emit('track',int(input()))
    # input()
breakpoint()