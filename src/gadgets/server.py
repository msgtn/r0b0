# from socketio import Server, AsyncServer()
# import eventlet
from src.config import GADGETS_DIR, STATIC_DIR, PUBLIC_DIR
from aiohttp import web
import socketio
# from socketio import 
from http import server as HTTPServer
import ssl


class Server(socketio.AsyncServer):
# class Server(socketio.Server):
    def __init__(self):
        super().__init__()
        self.on('midi',self.midi_event)
        
    def midi_event(self,sid, data):
        print(data, data)
     

async def index(request):
    with open(str(PUBLIC_DIR / 'index.html')) as f:
        return web.Response(text=f.read(), content_type='text/html')                                        
sio = Server()
app = web.Application()
app.router.add_static(str(STATIC_DIR), str(PUBLIC_DIR))
app.router.add_get(str(GADGETS_DIR),index)
sio.attach(app)
# app = socketio.WSGIApp(sio, static_files=static_files)

@sio.event
def connect(sid, environ, auth):
    print('connected')    

def main():
    web.run_app(app,
        host='localhost',
        port='8080')


if __name__=="__main__":
    main()
    