let peerConnection;
// const config = {
//   iceServers: [
//       { 
//         "urls": "stun:stun.l.google.com:19302",
//       },
//       // { 
//       //   "urls": "turn:TURN_IP?transport=tcp",
//       //   "username": "TURN_USERNAME",
//       //   "credential": "TURN_CREDENTIALS"
//       // }
//   ]
// };

const config = {
  iceServers: [{   urls: [ "stun:us-turn6.xirsys.com" ]}, {   username: "uTZru2n264RQqZAwDgHGuAt6tj8GV5cRJGQDrYMi65Hbogw1JEcFjVNnk9W6DzVIAAAAAF-Dv9Vwc3ljaG9tdWdz",   credential: "ee851dfc-0c32-11eb-a231-0242ac140004",   urls: [       "turn:us-turn6.xirsys.com:80?transport=udp",       "turn:us-turn6.xirsys.com:3478?transport=udp",       "turn:us-turn6.xirsys.com:80?transport=tcp",       "turn:us-turn6.xirsys.com:3478?transport=tcp",       "turns:us-turn6.xirsys.com:443?transport=tcp",       "turns:us-turn6.xirsys.com:5349?transport=tcp"   ]}]
};

const socket = io.connect(
  'https://c9268d392d2f.ngrok.io',
  // {transport: ['polling']}
  );
const video = document.querySelector("video");
const enableAudioButton = document.querySelector("#enable-audio");

enableAudioButton.addEventListener("click", enableAudio)

socket.on("offer", (id, description) => {
  peerConnection = new RTCPeerConnection(config);
  peerConnection
    .setRemoteDescription(description)
    .then(() => peerConnection.createAnswer())
    .then(sdp => peerConnection.setLocalDescription(sdp))
    .then(() => {
      socket.emit("answer", id, peerConnection.localDescription);
    });
  peerConnection.ontrack = event => {
    video.srcObject = event.streams[0];
  };
  peerConnection.onicecandidate = event => {
    if (event.candidate) {
      socket.emit("candidate", id, event.candidate);
    }
  };
});


socket.on("candidate", (id, candidate) => {
  peerConnection
    .addIceCandidate(new RTCIceCandidate(candidate))
    .catch(e => console.error(e));
});

socket.on("connect", () => {
  socket.emit("watcher");
});

socket.on("broadcaster", () => {
  socket.emit("watcher");
});

socket.on("disconnectPeer", () => {
  peerConnection.close();
});

window.onunload = window.onbeforeunload = () => {
  socket.close();
};

function enableAudio() {
  console.log("Enabling audio")
  video.muted = false;
}