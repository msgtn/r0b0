# from socketio import Server, AsyncServer()
import eventlet
from eventlet import wsgi
from src.config import GADGETS_DIR, STATIC_DIR, PUBLIC_DIR
from aiohttp import web
import asyncio
import socketio
# from socketio import 
from http import server as HTTPServer
import ssl

import time
# time.sleep()
# from flask import Flask
# from flask_restful import Api
# from flask.ext.socketio import SocketIO
# from flask_socketio import SocketIO

class Server(socketio.AsyncServer):
# class Server(socketio.Server):
        
    def __init__(self):
        super().__init__()
        # super().__init__(async_mode='eventlet')
        self.on('midi',self.midi_event)
        self.on('connect',self.connect_event)
        # time.sleep(5)
        # self.emit('midi',)
 
    def connect_event(self,sid,data):
        print(f"Connected to {sid}")
        # await self.emit('midi','fdfd','testing')
        # pass
        # return "OK", 123
        
    async def midi_event(self,sid, data):
    # async def midi_event(self,sid, data):
        time.sleep(2)
        await self.emit(
            'midi',
            data=data,
            to=sid
            )
        # self.emit('midi',data, to=sid)
        print('data',sid,data)
        # await self.emit('midi','tata',data)
        return "OK", 123
    
     

async def index(request):
    with open(str(PUBLIC_DIR / 'index.html')) as f:
        return web.Response(text=f.read(), content_type='text/html')                                        
sio = Server()

def main():
    # app = socketio.WSGIApp(sio)
    app = web.Application()
    app.router.add_static(str(STATIC_DIR), str(PUBLIC_DIR))
    app.router.add_get(str(GADGETS_DIR),index)
    sio.attach(app)

    web.run_app(app,
        host='localhost',
        port='8080')

def main_sync():
    static_files = {
        '/':{'content_type': 'text/html', 'filename':'index.html'}    
    }
    app = socketio.WSGIApp(sio, static_files)
    wsgi.server(eventlet.listen((
        'localhost', 8080)), app)

if __name__=="__main__":
    main_async()
    