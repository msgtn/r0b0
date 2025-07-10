import socketio

# Create a SocketIO client
sio = socketio.Client()

# Define the server address
# Replace with your server's address and port
SERVER_URL = "https://r0b0.ngrok.io:8080"

# Event handler for connection


@sio.event
def connect():
    print("Connected to the server")
    # Send a test message to the server
    sio.emit("test", {"message": "Hello from the client!"})

# Event handler for disconnection


@sio.event
def disconnect():
    print("Disconnected from the server")

# Event handler for custom events


@sio.on("response_event")
def on_response_event(data):
    print(f"Received response: {data}")


# Connect to the server
try:
    sio.connect(SERVER_URL)
    # Keep the client running to listen for events
    sio.wait()
except Exception as e:
    print(f"Failed to connect: {e}")
