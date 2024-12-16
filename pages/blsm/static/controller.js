/**
 * socketAddr and io (for sockets) are imported in the 
 * <script> tags in blsm_controller.html
 */

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


// const io = requirejs("static/socket.io")(server, {origins: '*:*'});
// const io = requirejs("/static/socket.io")(server, {origins: '*'});
var socket = "";
var backupSocket = "";
var lastEvent = 0;
var initYaw = 0;
var ctrlAddr = "";
var headCtrl = true;
var armsCtrl = true;

var yawOffset = 0;
var defaultHeight = 70;
var inEndpoint = false;
var showEndpointAlerts = true;
var endpointIndicator = document.getElementById("endpointIndicator");
var controlConsent = false;
var calibrateEveryControl = true;

var socket = null;
var backupSocket = null;

var controlBar = document.getElementById("controlBar");
// controlBar.opacity = 0.5;
var calibrateButton = document.getElementById("calibrate");
var heightControl = document.getElementById("heightControl");
var gestureBar = document.getElementById("gestureBar");
var broadcasterVideo = document.getElementById("broadcasterVideo");
var videoContainer = document.getElementById("videoContainer");
var videoInput = document.getElementById("videoInput");
var streamConstraints = {
  video: true,
  // audio: true,
  audio: false,
};

var controlSwitch = document
  .getElementById("controlSwitch")
  .querySelector("#control");
var appendageSwitch = document
  .getElementById("appendageSwitch")
  .querySelector("#control");
var mirrorSwitch = document
  .getElementById("mirrorSwitch")
  .querySelector("#mirror");
mirrorSwitch.checked = true;
// mirrorSwitch.checked = false;
var watcherVideo = document.getElementById("watcherVideo");
var audioSelect = document.getElementById("audioSource");
var videoSelect = document.getElementById("videoSource");
var recordingIndicator = document.getElementById("recordingIndicator");
var recordButton = document.getElementById("record");
var touchPad = document.getElementById("touchpad");
var tapeRow = document.getElementById("tapeRow");
var sourceRow = document.getElementById("sourceRow");
var wholeCover = document.getElementById("wholeCover");
// var tapeName = document.getElementById("tapeName");
var controlStuff = document.getElementById("controlStuff");
var loop = document.getElementById("loopSwitch"
).querySelector("#loop")
recordButton.classList.add("notRec");
recording = false;

// check if mobile device
var iosDevice = false;
var orientationEvent = "deviceorientation";
let mobileDevice =
  /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(
    navigator.userAgent
  );
if (mobileDevice) {
  if (typeof DeviceMotionEvent !== "undefined") {
    iosDevice = typeof DeviceMotionEvent.requestPermission === "function";
    orientationEvent = iosDevice
      ? "deviceorientation"
      : "deviceorientationabsolute";
    // alibrateButton.style.display = "none";
    // videoInput.style.display = "none";
    // appendageSwitch.style.display = "none";
    document.getElementById("appendageSwitch").style.display = "none";

  }
} else {
  controlBar.style.display = "none";
  // calibrateButton.style.display = "none";
  // touchPad.style.display = "none";
}
touchPad.style.display = "none";
// tapeRow.style.display = "none";
sourceRow.style.display = "none";
controlStuff.style.position = "absolute";
controlStuff.style.bottom = 0;
controlStuff.style.display = "flex";
// controlStuff.style.display = "flex";
indicatorContainer = document.getElementById("indicatorContainer")
indicatorContainer.style.display = "none";
endpointIndicator.style.display = "none";


let tailHeight = 0.2;

var enableHeightControl = true;

function moveTouch(event) {
  function drawSliders() {
    // ctx.clearRect(0,0,touchPad.width,2*touchPad.height);

    ctx.putImageData(blankTouchPad, 0, 0);
    ctx.beginPath();

    let sliderArray = enableHeightControl
      ? [leftSlider, heightSlider, rightSlider]
      : [leftSlider, rightSlider];
    let colWidth = window.innerWidth / (enableHeightControl ? 3 : 2);
    sliderArray.forEach((y, i) => {
      ctx.moveTo(colWidth * i, y);
      ctx.lineTo(colWidth * (i + 1), y);
      ctx.stroke();
    });
    ctx.moveTo(tailSlider, touchPad.height * (1 - tailHeight));
    ctx.lineTo(tailSlider, touchPad.height);
    ctx.stroke();
  }

  for (const touchIdx in event.targetTouches) {
    if (isNaN(touchIdx)) {
      continue;
    }

    var touchVal = event.targetTouches[touchIdx];
    var halfSubDiv = subDiv / 2;
    // control left arm
    touchPadOffset = -200;
    touchPadOffset = 0;
    if (touchVal["screenY"] < touchPad.height * (1 - tailHeight)) {
      if (touchVal["screenX"] < window.innerWidth / halfSubDiv) {
        leftArmPos = -((touchVal["screenY"] / touchPad.height) * 300) + 150;
        leftSlider = touchVal["screenY"];
        // control height (may be disabled)
      } else if (
        enableHeightControl &
        (touchVal["screenX"] <
          (window.innerWidth / halfSubDiv) * (halfSubDiv - 1))
      ) {
        heightPos = (1 - touchVal["screenY"] / touchPad.height) * 100 - 25;
        heightSlider = touchVal["screenY"];
        // control right arm
      } else if (
        touchVal["screenX"] >
        (window.innerWidth / halfSubDiv) * (halfSubDiv - 1)
      ) {
        rightArmPos = (touchVal["screenY"] / touchPad.height) * 300 - 150;
        rightSlider = touchVal["screenY"];
      }
    } else {
      tailSlider = touchVal["screenX"];
      tailPos = -((touchVal["screenX"] / window.innerWidth) * 200 - 100);
      console.log(touchVal["screenX"], window.innerWidth, tailSlider);
    }
    drawSliders();
  }

  // TODO - add sliders
}
// touchPad.ontouchmove = moveTouch;

const peerConnections = {};

function startup() {
  broadcasterVideo.width = parseInt(window.innerWidth) * 1;
  broadcasterVideo.height = (broadcasterVideo.width * 3) / 4;
  watcherVideo.width = watcherVideo.height = 0;
  videoContainer.width = broadcasterVideo.width;
  videoContainer.height = broadcasterVideo.height;
  watcherVideo.style.zIndex = "-1";
  broadcasterVideo.style.zIndex = "-1";

  endpointIndicator.style.top = broadcasterVideo.height / 3 + "px";
  recordingIndicator.style.top = watcherVideo.height / 10 + "px";
  recordingIndicator.style.textAlign = "left";
  recordingIndicator.style.left = "50%";
  recordingIndicator.innerHTML = "";
  controlSwitch.disabled = false;
  // calibrateButton.disabled = false;
  // recordButton.disabled = false;

  // TODO - set this to a yaml config
  socket = io.connect(socketAddr);
  backupSocket = io.connect(socketAddr);

  let peerConnection;

  socket.on("test", () => {
    console.log("received test");
  });

  socket.on("stopControl", () => {
    controlSwitch.checked = false;
    onControl();
  });


  socket.on("connect", () => {
    socket.emit("watcher", socket.id);
  });

  socket.on("broadcaster", () => {
    socket.emit("watcher", socket.id);
  });

  socket.on("candidate", (id, candidate) => {
    console.log(id);
    if (id in peerConnections) {
      peerConnections[id]
        .addIceCandidate(new RTCIceCandidate(candidate))
        .catch((e) => console.error(e));
    }
  });

  socket.on("offer", async (id, description) => {
    console.log("Got offer");
    peerConnection = new RTCPeerConnection(config);
    peerConnections[id] = peerConnection;

    // const stream = await navigator.mediaDevices.getUserMedia(streamConstraints);

    // stream.getTracks().forEach((track) => {
    //   peerConnection.addTrack(track, stream);
    // });

    peerConnection
      .setRemoteDescription(description)
      .then(() => peerConnection.createAnswer())
      .then((sdp) => peerConnection.setLocalDescription(sdp))
      .then(() => {
        socket.emit("answer", id, peerConnection.localDescription);
      });
    peerConnection.ontrack = (event) => {
      broadcasterVideo.srcObject = event.streams[0];
      // broadcasterVideo.srcObject = event.streams[];
      console.log(event.streams);
      console.log(event.streams[0]);
    };
    peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        socket.emit("candidate", id, event.candidate);
      }
    };
  });

  socket.on("python", (data) => {
    console.log(data);
  });

  socket.on("answer", (id, description) => {
    peerConnections[id].setRemoteDescription(description);
    peerConnections[id].ontrack = (event) => {
      broadcasterVideo.srcObject = event.streams[0];
      console.log(event.streams[0]);
    };
  });

  socket.on("disconnectPeer", (id) => {
    peerConnections[id].close();
    delete peerConnections[id];
  });
}

function compassHeading(alpha, beta, gamma) {
  var degtorad = Math.PI / 180; // Degree-to-Radian conversion

  var _x = beta ? beta * degtorad : 0; // beta value
  var _y = gamma ? gamma * degtorad : 0; // gamma value
  var _z = alpha ? alpha * degtorad : 0; // alpha value

  var cX = Math.cos(_x);
  var cY = Math.cos(_y);
  var cZ = Math.cos(_z);
  var sX = Math.sin(_x);
  var sY = Math.sin(_y);
  var sZ = Math.sin(_z);

  // Calculate Vx and Vy components
  var Vx = -cZ * sY - sZ * sX * cY;
  var Vy = -sZ * sY + cZ * sX * cY;

  // Calculate compass heading
  var compassHeading = Math.atan(Vx / Vy);

  // Convert compass heading to use whole unit circle
  if (Vy < 0) {
    compassHeading += Math.PI;
  } else if (Vx < 0) {
    compassHeading += 2 * Math.PI;
  }
  // return compassHeading;
  return compassHeading * (180 / Math.PI); // Compass Heading (in degrees)
}

const handleOrientation = (e) => {
  // if (!controlSwitch.checked && !appendageSwitch.checked) {
  //   return;
  // }

  beta = (e.beta * Math.PI) / 180;
  gamma = (e.gamma * Math.PI) / 180;

  alphaTrue = compassHeading(e.alpha, e.beta, e.gamma);
  alpha = ((alphaTrue - yawOffset) * Math.PI) / 180;

  // constraint 0 < alpha < 2pi
  if (alpha < 0) {
    alpha += 2 * Math.PI;
  }
  if (alpha > 2 * Math.PI) {
    alpha -= 2 * Math.PI;
  }

  var endpointThreshold = Math.PI / 6;
  var distToEndpoint = Math.abs(Math.PI - alpha);
  // beta<2 because sometimes indicates when just looking up
  // doesnt catch high beta in the endpoints anyways
  // only catches if pitch is not much higher than the horizon
  if (!inEndpoint && distToEndpoint < endpointThreshold && beta < 2) {
    inEndpoint = true;
    if (Math.PI > alpha) {
      endpointIndicator.style.textAlign = "left";
      endpointIndicator.style.left = "10%";
      endpointIndicator.innerHTML = "<";
    } else {
      endpointIndicator.style.textAlign = "right";
      endpointIndicator.style.left = "90%";
      endpointIndicator.innerHTML = ">";
    }
    if (!viewSwitch.checked && showEndpointAlerts) {
      // showEndpointAlerts = confirm("Hit endpoint, turn around!\nPress Cancel to block future popups.");
    }
    // increase endpoint to make it harder to get out of it
    // this should reduce the frequency of alerts
  } else if (inEndpoint && distToEndpoint > 2 * endpointThreshold) {
    inEndpoint = false;
    endpointIndicator.innerHTML = " ";
  }

  // if (e.timeStamp - lastEvent > 50) {
  if (e.timeStamp - lastEvent > 100) {
    lastEvent = e.timeStamp;
    let _time = Date.now();
    let datetime = new Date();
    let _datetime = datetime.toISOString();
    var body = {
      event: "device_motion",
      x: parseFloat(beta.toFixed(2)),
      y: parseFloat(gamma.toFixed(2)),
      z: parseFloat(alpha.toFixed(2)),
      // x: beta,
      // y: gamma,
      // z: alpha,
      ax: 0,
      ay: 0,
      az: 0,
      h: defaultHeight,
      // h: parseInt(heightPos),
      // h: parseInt(heightPos),
      q: 0,
      ears: 50,
      // left_arm: -leftArmPos,
      // right_arm: -rightArmPos,
      // tail: tailPos,
      landscape: false,
      // mirror: !externalCam,
      // mirror: true, # this is actually unmirrored (fpv)
      // mirror: false,  // this is actually mirrored (3pv)
      mirror: !mirrorSwitch.checked,
      // mirror: false,
      heightCtrl: false,
      // armsCtrl: appendageSwitch.checked,
      headCtrl: controlSwitch.checked,
      yaw: 0,
      time: _time,
      datetime: _datetime,
      portrait: true,
      id: socket.id,
    };

    socket.emit("device_motion", body);
    // backupSocket.emit("device_motion", body);
    // socket.emit("message", {data: 'test'});
    console.log(body);
  }
};

function calibrateYaw() {
  // heightSlider.value = "70";
  fetch(ctrlAddr + `/reset`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
  })
    .then(() => {
      yawOffset = alphaTrue;
    })
    .catch(() => {
      console.log("Couldnt calibrate yaw");
    });
}

function onControl() {
  controlConsent = true;
  // if (controlSwitch.checked || appendageSwitch.checked) {
  if (controlSwitch.checked) {
    if (controlSwitch.checked && calibrateEveryControl) {
      calibrateYaw();
    }
    stopTape();
    // feature detect
    if (iosDevice) {
      DeviceMotionEvent.requestPermission()
        .then((permissionState) => {
          if (permissionState === "granted") {
            window.addEventListener(orientationEvent, handleOrientation, false);
          }
        })
        .catch(console.error);
    } else {
      if (window.DeviceMotionEvent) {
        window.addEventListener(orientationEvent, handleOrientation, false);
      }
    }
  } else {
    window.removeEventListener(orientationEvent, handleOrientation);
  }
}

function onRecord() {
  console.log("record");
  // socket.emit("record",{record:true,event:'device_motion'});
  recording = !recording;
  controlSwitch.checked = recording;
  onControl();
  recordButton.classList.remove(recording ? "notRec" : "Rec");
  recordButton.classList.add(recording ? "Rec" : "notRec");
  socket.emit("record", {
    record: recording,
    event: "device_motion",
    id: socket.id,
  });
  setTimeout(updateTapes(), 2000);
}

function onLoad() {
  console.log("load");
  socket.emit("load", {
    tapeName: tapeName.value,
    event: "load",
    id: socket.id,
  });
}

window.onunload = window.onbeforeunload = () => {
  socket.close();
};

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
      var videoOptions = [...videoSelect.options].map((o) => o.label);
      if (!videoOptions.includes(deviceInfo.label)) {
        videoSelect.appendChild(option);
      }
    }
  }
}

function getStream() {
  console.log("Getting stream");
  const audioSource = audioSelect.value;
  const videoSource = videoSelect.value;
  streamConstraints = {
    audio: { deviceId: audioSource ? { exact: audioSource } : undefined },
    video: {
      deviceId: videoSource ? { exact: videoSource } : undefined,
      // frameRate: { min: 30 },
    },
  };
  return navigator.mediaDevices
    .getUserMedia(streamConstraints)
    .then(gotStream)
    .catch(handleError);
}

function gotStream(stream) {
  window.stream = stream;
  audioSelect.selectedIndex = [...audioSelect.options].findIndex(
    (option) => option.text === stream.getAudioTracks()[0].label
  );
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    (option) => option.text === stream.getVideoTracks()[0].label
  );

  Object.keys(peerConnections).forEach(function (id) {
    stream.getTracks().forEach((track) => {
      peerConnections[id].addTrack(track, stream);
    });
  });
}

function handleError(error) {
  console.error("Error: ", error);
}

audioSelect.onchange = getStream;
videoSelect.onchange = getStream;

getStream(audioSelect).then(getDevices).then(gotDevices);
getStream(videoSelect).then(getDevices).then(gotDevices);

function onText() {
  controlSwitch.checked = false;
  onControl();
  let res = window.prompt("Type a message to Blossom");
  if (res == null || res == "") {

  } else {
    socket.emit("phone_text", {
      event: "phone_text",
      text: res,
      id: socket.id,
    });
    console.log("emitted onText");
  }
}

// const socket = io.connect(socketAddr)
const tapesSelect = document.getElementById("tapes");
// const loop = document.getElementById("loop");


function updateTapes() {
  fetch(`${socketAddr}/tapes`)
    .then(response => response.json())
    .then(data => {
      console.log(tapesSelect);
      console.log(data);
      data = data.reverse();
      data.map((lang, i) => {
        let opt = document.createElement("option");
        opt.value = i; // the index
        opt.innerHTML = lang.replace(".json", "");
        console.log(opt);
        // tapesSelect.append(opt);
        tapesSelect[i] = opt;
      })
    })
    ;
}

function playTape() {
  controlSwitch.checked = false; onControl();
  socket.emit("play",
    {
      'tape_name': selectedTape(),
      'loop': loop.checked,
    }
  )
}

function selectedTape() {
  let tapesSelect = document.getElementById("tapes");
  return tapesSelect.options[tapesSelect.selectedIndex].text;
}

function stopTape() {
  // controlSwitch.checked = false; onControl();
  socket.emit("stop",
    {
      "tape_name": selectedTape(),
    }
  )
}

if (false) {
  wholeCover.style.display = "none";
  // tapeRow.style.display = "none";
  // controlStuff.style.display = "none";
  let res = window.prompt("Please type the 4-digit code:");
  if (res == null || res == "") {

  } else {
    if (res == "1234") {
      wholeCover.style.display = "inline";
      // tapeRow.style.display = "inline";
      // controlStuff.style.display = "inline";

      startup();

      updateTapes();
    }
  }
}

// wholeCover.style.display = "inline";
startup();
updateTapes();
