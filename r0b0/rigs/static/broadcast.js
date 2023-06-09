
const peerConnections = {};


const config = {
  iceServers: [
    {   urls: [ "stun:us-turn6.xirsys.com" ]},
    {   username: "uTZru2n264RQqZAwDgHGuAt6tj8GV5cRJGQDrYMi65Hbogw1JEcFjVNnk9W6DzVIAAAAAF-Dv9Vwc3ljaG9tdWdz",   credential: "ee851dfc-0c32-11eb-a231-0242ac140004",   urls: [       "turn:us-turn6.xirsys.com:80?transport=udp",       "turn:us-turn6.xirsys.com:3478?transport=udp",       "turn:us-turn6.xirsys.com:80?transport=tcp",       "turn:us-turn6.xirsys.com:3478?transport=tcp",       "turns:us-turn6.xirsys.com:443?transport=tcp",       "turns:us-turn6.xirsys.com:5349?transport=tcp"   ]}]
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
  const peerConnection = new RTCPeerConnection(config);
  peerConnections[id] = peerConnection;
  // peerConnection = peerConnections[id];
  const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });
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
      // console.log(id, peerConnection.localDescription);
      socket.emit("offer", id, peerConnection.localDescription);
    })
    .catch(() => {
      console.log("Error offering")
    });
});


socket.on("answer", (id, description) => {
  peerConnections[id].setRemoteDescription(description);
});

socket.on("broadcaster", () => {
  // necessary to get controller's video feed
  // socket.emit("watcher",socket.id);
});


socket.on("offer", (id, description) => {
  console.log("Got offer");
  console.log(id)
  console.log(description);
  peerConnection = new RTCPeerConnection(config);
  // peerConnections[id] = peerConnection;
  peerConnection
    .setRemoteDescription(description)
    .then(() => peerConnection.createAnswer())
    .then(sdp => peerConnection.setLocalDescription(sdp))
    .then(() => {
      socket.emit("answer", id, peerConnection.localDescription);
    });
  console.log("Set peer connection");
  peerConnection.ontrack = event => {
    watcherVideo.srcObject = event.streams[0];
    console.log(event.streams[0]);

  };
  console.log("Set ontrack");
  peerConnection.onicecandidate = event => {
    if (event.candidate) {
      socket.emit("candidate", id, event.candidate);
    }
  };
  console.log("Set onicecandidate")
});


socket.on("candidate", (id, candidate) => {
  console.log(id,candidate);
  peerConnections[id].addIceCandidate(new RTCIceCandidate(candidate));
});

socket.on("disconnectPeer", id => {
  peerConnections[id].close();
  delete peerConnections[id];
});

// commanded by user to switch cameras
socket.on("switchCam", (flatSwitchChecked) => {
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    option => option.text === camList[flatSwitchChecked ? 0 : 1]
  );
  getStream(videoSelect)
    .then(getDevices)
    .then(gotDevices)
    .then(socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text));
})

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
    audio: { deviceId: audioSource ? { exact: audioSource } : undefined },
    video: { deviceId: videoSource ? { exact: videoSource } : undefined , frameRate: {min:30}}
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
  // socket.emit("watcher");
  console.log("Got stream");
  console.log(`gotStream ${videoSelect.options[videoSelect.selectedIndex].text}`);
  socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);

}



function handleError(error) {
  console.error("Error: ", error);
}