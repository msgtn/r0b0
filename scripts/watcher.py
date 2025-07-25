import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.signaling import TcpSocketSignaling, UnixSocketSignaling, object_from_string
from aiortc.contrib.media import MediaPlayer, MediaRecorder
from av import VideoFrame
import cv2

# Configuration for the ICE servers
ICE_SERVERS = [
    {
        "urls": [
            "stun:us-turn8.xirsys.com",
            "turn:us-turn8.xirsys.com:80?transport=udp",
            "turn:us-turn8.xirsys.com:3478?transport=udp",
            "turn:us-turn8.xirsys.com:80?transport=tcp",
            "turn:us-turn8.xirsys.com:3478?transport=tcp",
            "turns:us-turn8.xirsys.com:443?transport=tcp",
            "turns:us-turn8.xirsys.com:5349?transport=tcp",
        ],
        "username": "LGTr4T-fYrwaB75qalVpHmjJshsXmRmWbz5fScJHQG9aQ0i_2DqL_0LF6MIScX31AAAAAGSwsDdtc3VndWl0YW4=",
        "credential": "94513d7a-21ec-11ee-902b-0242ac140004",
    }
]

async def run_watcher():
    # Create a peer connection
    pc = RTCPeerConnection({"iceServers": ICE_SERVERS})

    # Connect to the signaling server
    # signaling = TcpSocketSignaling("localhost", 5000)  # Replace with your signaling server address
    # signaling = TcpSocketSignaling("0.0.0.0", 8080)
    # signaling = UnixSocketSignaling("https://r0b0.ngrok.io")
    signaling = TcpSocketSignaling("localhost", 8080)
    await signaling.connect()

    await signaling.send(object_from_string(json.dumps({"type":"watcher"})))
    # Handle incoming tracks
    @pc.on("track")
    def on_track(track):
        print(f"Receiving {track.kind} track")
        if track.kind == "video":
            # Display the video stream
            async def display_video():
                while True:
                    frame = await track.recv()
                    img = frame.to_ndarray(format="bgr24")
                    cv2.imshow("Watcher Video", img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                cv2.destroyAllWindows()

            asyncio.ensure_future(display_video())

    # Wait for an offer from the broadcaster
    print("Waiting for offer...")
    offer = await signaling.receive()
    if isinstance(offer, RTCSessionDescription) and offer.type == "offer":
        await pc.setRemoteDescription(offer)

        # Create and send an answer
        await pc.setLocalDescription(await pc.createAnswer())
        await signaling.send(pc.localDescription)

    # Handle ICE candidates
    while True:
        obj = await signaling.receive()
        if isinstance(obj, RTCIceCandidate):
            await pc.addIceCandidate(obj)
        elif obj is None:
            break

    # Close the connection when done
    await pc.close()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run_watcher())