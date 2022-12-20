# from socketio import Server, AsyncServer()
# import eventlet
from aiohttp import web
import socketio
from http import server as HTTPServer
import ssl

class Server(socketio.AsyncServer):
    def __init__(self):
        super().__init__()

async def index(request):
    with open('./public/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

sio = Server()
app = web.Application()
app.router.add_static('/static','./public')
app.router.add_get('/',index)

@sio.event
def connect(sid, environ, auth):
    print('connected', sid)

# async def main():
#     # web.run_app(app)
#     runner = web.AppRunner(app)
#     await runner.setup()
#     site = web.TCPSite(runner, 'localhost', 8080)
#     await site.start()
#     input()

if __name__=="__main__":
    # main()
    web.run_app(app)
