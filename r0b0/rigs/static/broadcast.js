const broadcastAudio = false;
const peerConnections = {};
const config = {
  iceServers: [
    { urls: ["stun:us-turn8.xirsys.com"] },
    {
      username:
        "LGTr4T-fYrwaB75qalVpHmjJshsXmRmWbz5fScJHQG9aQ0i_2DqL_0LF6MIScX31AAAAAGSwsDdtc3VndWl0YW4=",
      credential: "94513d7a-21ec-11ee-902b-0242ac140004",
      urls: [
        "turn:us-turn8.xirsys.com:80?transport=udp",
        "turn:us-turn8.xirsys.com:3478?transport=udp",
        "turn:us-turn8.xirsys.com:80?transport=tcp",
        "turn:us-turn8.xirsys.com:3478?transport=tcp",
        "turns:us-turn8.xirsys.com:443?transport=tcp",
        "turns:us-turn8.xirsys.com:5349?transport=tcp",
      ],
    },
  ],
};


// Get camera and microphone
const broadcasterVideo = document.getElementById("broadcasterVideo");
var watcherVideo = document.getElementById("watcherVideo");
const audioSelect = document.getElementById('audioSource');
const videoSelect = document.getElementById('videoSource');

const socket = io.connect(window.location.origin);
console.log(socket.id);

socket.on("connect", () => {
  console.log("Connecting");
  // socket.emit("watcher",socket.id);
  // socket.emit("broadcaster",socket.id);
});

function sendWatcher() {
  // socket.emit("watcher",socket.id);
  setTimeout(function() {
    socket.emit("broadcaster",socket.id);
  }, 1000);
};

socket.on("watcher", async (id) => {
  console.log(`watcher from ${id}`)
  const peerConnection = new RTCPeerConnection(config);
  peerConnections[id] = peerConnection;
  // peerConnection = peerConnections[id];
  const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: broadcastAudio });
  stream.getTracks().forEach((track) => {peerConnection.addTrack(track, stream)});
  peerConnection.onicecandidate = event => {
    if (event.candidate) {
      socket.emit("candidate", id, event.candidate);
    }
  };
  peerConnection.ontrack = event => {
    watcherVideo.srcObject = event.streams[0];
    console.log(event.streams[0]);
  };

  peerConnection
    .createOffer()
    .then(sdp => peerConnection.setLocalDescription(sdp))
    .then(() => {
      socket.emit("offer", id, peerConnection.localDescription);
    })
    .catch(() => {
      console.log("Error offering")
    });
});

socket.on("candidate", (id, candidate) => {
  console.log(id,candidate);
  peerConnections[id].addIceCandidate(new RTCIceCandidate(candidate));
});

socket.on("answer", (id, description) => {
  peerConnections[id].setRemoteDescription(description);
});
socket.on("disconnectPeer", id => {
  peerConnections[id].close();
  delete peerConnections[id];
});

window.onunload = window.onbeforeunload = () => {
  socket.close();
};

audioSelect.onchange = getStream;
videoSelect.onchange = getStream;

getStream(videoSelect)
  .then(getDevices)
  .then(gotDevices);

function getDevices() {
  return navigator.mediaDevices.enumerateDevices();
}

function gotDevices(deviceInfos) {
  window.deviceInfos = deviceInfos;
  for (const deviceInfo of deviceInfos.reverse()) {
    const option = document.createElement("option");
    option.value = deviceInfo.deviceId;
    if (deviceInfo.kind === "audioinput") {
      option.text = deviceInfo.label || `Microphone ${audioSelect.length + 1}`;
      audioSelect.appendChild(option);
    } else if (deviceInfo.kind === "videoinput") {
      option.text = deviceInfo.label || `Camera ${videoSelect.length + 1}`;
      var videoOptions = [...videoSelect.options].map(o => o.label);
      if (!videoOptions.includes(deviceInfo.label)) {
        videoSelect.appendChild(option);
      };
    }
  }
}

function getStream() {
  if (window.stream) {
    window.stream.getTracks().forEach(track => {
      track.stop();
    });
  }
  const audioSource = audioSelect.value;
  const videoSource = videoSelect.value;
  const constraints = {
    // audio: { deviceId: audioSource ? { exact: audioSource } : undefined },
    video: { deviceId: videoSource ? { exact: videoSource } : undefined , 
      frameRate: {min:30}
    }
  };
  return navigator.mediaDevices
    .getUserMedia(constraints)
    .then(gotStream)
    .catch(handleError);
}

function gotStream(stream) {
  window.stream = stream;
  // window.stream.applyConstraints({ frameRate: { min: 30 } });
  audioSelect.selectedIndex = [...audioSelect.options].findIndex(
    option => option.text === stream.getAudioTracks()[0].label
  );
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    option => option.text === stream.getVideoTracks()[0].label
  );
  broadcasterVideo.srcObject = stream;
  socket.emit("broadcaster",socket.id);
  
  console.log("Got stream");
}

function handleError(error) {
  console.error("Error: ", error);
}