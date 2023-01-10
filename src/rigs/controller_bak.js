
// import { DeviceOrientationControls } from "./DeviceOrientationControls.js";

// document.getElementById("popup").style.display = "block";

// var password = prompt("Enter login ID: ");

// - [ ] https://dev.to/li/how-to-requestpermission-for-devicemotion-and-deviceorientation-events-in-ios-13-46g2
// - [ ] https://developer.mozilla.org/ja/docs/Web/API/DeviceMotionEvent
// - [ ] https://triple-underscore.github.io/deviceorientation-ja.html

// import {} from 
// import { dataSet } from './data.js'
console.log(dataSet)

var evalMode = true;
var enableViewSwitch = false;
var enableGestureBar = false;

var ctrlSocket = "";
var lastEvent = 0;
var initYaw = 0;
var ctrlAddr = "";
var headCtrl = true;
var armsCtrl = true;

var alphaTrue = 0;
var yawOffset = 0;
var defaultHeight = 70;
var inEndpoint = false;
var showEndpointAlerts = true;
var leftArmPos = -100;
var rightArmPos = 100;
var tailPos = 0;
var heightPos = 50;
var endpointIndicator = document.getElementById("endpointIndicator");
// var heightSlider = document.getElementById("heightSlider");
var controlConsent = false;
var calibrateEveryControl = true;

var recordButton = document.getElementById("record");
recordButton.classList.add("notRec");
var recordMode = "start";


const ytVid = document.getElementById("ytVid");
const videoButton = document.getElementById("displayVideoById");
const videoId = document.getElementById("videoId");

const userNamesGesture = !evalMode;
const survey = document.getElementById("survey");
var gestures = document.getElementById("gestures");
var gestureList = [];

var socket = null;


// dictionary of videos and IDs

function swap(json){
  var ret = {};
  for(var key in json){
    ret[json[key]] = key;
  }
  return ret;
}
// reverse the dictionary
var ytIdDict = swap(ytDict);
var curVideo = ""
var curId = ""

// 0=laptop (third-person), 1=external (first-person)
var externalCam = true;

var controlBar = document.getElementById("controlBar");
var calibrateButton = document.getElementById("calibrate");
var heightControl = document.getElementById("heightControl");
var gestureBar = document.getElementById("gestureBar");
var broadcasterVideo = document.getElementById("broadcasterVideo");
var videoContainer = document.getElementById("videoContainer");
var videoInput = document.getElementById("videoInput");
// var viewSwitch = document.getElementById("viewSwitch");
var touchPad = document.getElementById("touchpad");
var playGestureButton = document.getElementById("playGesture");
var controlSwitch = document.getElementById("controlSwitch").querySelector("#control");
var appendageSwitch = document.getElementById("appendageSwitch").querySelector("#control");
// var viewSwitch = document.getElementById("viewSwitch").querySelector("#view");
var watcherVideo = document.getElementById("watcherVideo");
var audioSelect = document.getElementById('audioSource');
var videoSelect = document.getElementById('videoSource');
var recordingIndicator = document.getElementById("recordingIndicator");
var promptSpace = document.getElementById("promptSpace");
var videoWebm = document.getElementById("videoWebm");
var playWebmButton = document.getElementById("playWebm");
var curEmbodiment = "";

// check if mobile device
var mobileDevice = (/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent));
const iosDevice =  typeof DeviceMotionEvent.requestPermission === "function";
const orientationEvent = iosDevice ? "deviceorientation" : "deviceorientationabsolute";

// hide controllers if not on a mobile device

if (mobileDevice) {
  calibrateButton.style.display = "none";
  videoInput.style.display = "none";
} else {
  controlBar.style.display = "none";
  calibrateButton.style.display = "none";
  heightControl.style.display = "none";
  gestureBar.style.display = "none";
  videoId.width = window.innerWidth / 2;
  videoButton.width = window.innerWidth / 2;
  videoSelect.style.display = "none";
  videoInput.style.display = "none";
  touchPad.style.display = "none";
  playWebmButton.style.display = "none";

}
videoSelect.style.display = "none";
playWebmButton.style.display = "none";
promptSpace.style.display = "none";
watcherVideo.style.display = "none";
videoWebm.style.display = "none";

// document.getElementById("viewSwitch").style.visibility = enableViewSwitch ? "initial":"hidden"
gestureBar.style.display = enableGestureBar ? "initial":"none"
heightControl.style.display = "none";

touchPad.height = window.innerHeight*0.85;
touchPad.width = window.innerWidth;
let tailHeight = 0.2

var enableHeightControl = true

var ctx = touchPad.getContext("2d");
ctx.font = "20px Arial";
ctx.textAlign="center"
var subDiv = enableHeightControl ? 6 : 4
if (enableHeightControl) { ctx.fillText("HEIGHT", window.innerWidth/2, 50); };
ctx.fillText("LEFT ARM", window.innerWidth/subDiv, 50);
ctx.fillText("RIGHT ARM", window.innerWidth/subDiv*(subDiv-1), 50);
for (i=1; i<subDiv/2; i++) {
  ctx.moveTo(window.innerWidth/(subDiv/2)*i,0);
  ctx.lineTo(window.innerWidth/(subDiv/2)*i,touchPad.height*(1-tailHeight));
  ctx.stroke();
}
ctx.moveTo(0,touchPad.height*(1-tailHeight));
ctx.lineTo(window.innerWidth,touchPad.height*(1-tailHeight));
ctx.stroke();

ctx.fillText("TAIL", window.innerWidth/2, touchPad.height*(1-tailHeight)+35);

let blankTouchPad = ctx.getImageData(0,0,touchPad.width,touchPad.height);

// let rightSlider, leftSlider = 0,0;
let rightSlider = 0;
let leftSlider = 0;
let heightSlider = 0;
let tailSlider = 0;

touchPad.addEventListener("touchmove", function(event) {
  event.preventDefault();
}, false); 

function moveTouch(event) {
  function drawSliders() {
    // ctx.clearRect(0,0,touchPad.width,2*touchPad.height);

    ctx.putImageData(blankTouchPad,0,0);
    ctx.beginPath();

    let sliderArray = enableHeightControl ? [leftSlider, heightSlider, rightSlider] : [leftSlider, rightSlider];
    let colWidth = window.innerWidth / (enableHeightControl ? 3 : 2);
    sliderArray.forEach( (y,i) => {
      // ctx.moveTo(window.innerWidth/(subDiv/2),y);
      // ctx.moveTo(window.innerWidth/(subDiv/(enableHeightControl ? 3 : 2)),y);
      // ctx.moveTo(window.innerWidth*(i+1)/(enableHeightControl ? 3 : 2),y);
      // ctx.lineTo(i * window.innerWidth, y);
      ctx.moveTo(colWidth*i,y);
      ctx.lineTo(colWidth*(i+1),y);
      ctx.stroke();
    });
    ctx.moveTo(tailSlider, touchPad.height*(1-tailHeight));
    ctx.lineTo(tailSlider, touchPad.height);
    ctx.stroke();
  }

  for (const touchIdx in event.targetTouches) {

    if (isNaN(touchIdx)) { continue; };
    
      var touchVal = event.targetTouches[touchIdx];
      var halfSubDiv = subDiv/2;
      // control left arm
      touchPadOffset = -200;
      touchPadOffset = 0;
      if (touchVal["screenY"]<touchPad.height*(1-tailHeight)) {
        if (touchVal["screenX"]<(window.innerWidth/halfSubDiv)) {
          leftArmPos = -(touchVal["screenY"]/touchPad.height*300)+150;
          leftSlider = touchVal["screenY"];
        // control height (may be disabled)
        } else if (enableHeightControl & touchVal["screenX"]<(window.innerWidth/halfSubDiv*(halfSubDiv-1))) {
          heightPos = (1-touchVal["screenY"]/touchPad.height)*100-25
          heightSlider = touchVal["screenY"]
        // control right arm
        } else if (touchVal["screenX"]>(window.innerWidth/halfSubDiv*(halfSubDiv-1))) {
          rightArmPos = (touchVal["screenY"]/touchPad.height*300)-150;
          rightSlider = touchVal["screenY"];
        }
      } else {
        tailSlider = touchVal["screenX"];
        tailPos = -((touchVal["screenX"]/window.innerWidth)*200-100);
        console.log(touchVal["screenX"], window.innerWidth, tailSlider);
      }
      drawSliders();
  }

  // TODO - add sliders
} 
touchPad.ontouchmove = moveTouch;

function playWebm() {
  videoWebm.currentTime = 0;
  socket.emit("playWebm"); 
  setTimeout(function() {
    videoWebm.play();
  }, 600);
}

const peerConnections = {};

function checkPswd() {
  // heightSlider.style.width = "100%";
  // broadcasterVideo.width = parseInt(window.innerWidth/2);
  // broadcasterVideo.height = broadcasterVideo.width*3/4;
  broadcasterVideo.width = parseInt(window.innerWidth)*1;
  broadcasterVideo.height = broadcasterVideo.width*3/4;
  // watcherVideo.width = broadcasterVideo.width;
  // watcherVideo.height = watcherVideo.width*3/4;
  watcherVideo.width = watcherVideo.height = 0;
  // video.height = 0;
  videoContainer.width = broadcasterVideo.width;
  videoContainer.height = broadcasterVideo.height;
  watcherVideo.style.zIndex = "-1";
  broadcasterVideo.style.zIndex = "-1";

  endpointIndicator.style.top = broadcasterVideo.height/3+"px";
  recordingIndicator.style.top = watcherVideo.height/10+"px";
  recordingIndicator.style.textAlign="left";
  recordingIndicator.style.left="50%";
  recordingIndicator.innerHTML = "";
  controlSwitch.disabled = false;
  appendageSwitch.disabled = false;
  // viewSwitch.disabled = evalMode;
  calibrateButton.disabled = false;
  playGestureButton.disabled = false;
  recordButton.disabled = false;

  var socketAddr = "blossomapp.ngrok.io"
  if (document.location.href.includes('localhost')) {
    socketAddr = "localhost:8000";
    console.log("Using localhost socket")
  }

  fetch(`https://${socketAddr}/ctrl_addr.txt`)
    .then(response => response.text())
    .then((data) => {
      ctrlAddr = data;
      ctrlSocket = io.connect(ctrlAddr);
      if (mobileDevice) { getGestures(); };
    })
    .catch(() => {
      console.log("Couldnt get control address");
      socketAddr = "localhost:8000";
    })

  socket = io.connect(`https://${socketAddr}`);

  let peerConnection;

  socket.on("updateRecInd", (text) => {
    recordingIndicator.innerHTML = text;
  });

  socket.on("showWebm", (showWebm) => {
    console.log("Showing Webm");
    videoWebm.width = showWebm ? broadcasterVideo.width : 0;
    videoWebm.height = videoWebm.width*3/4; 
    playWebmButton.style.display = showWebm ? "block" : "none";
    videoWebm.load();
  });

  socket.on("showPrompt", (showPrompt) => {
    promptSpace.innerHTML = showPrompt;
    promptSpace.innerText = showPrompt;
  });


  socket.on("watcher", async (id) => {
    console.log("Watcher");
    // console.log(`watcher ${camList[vidViewDict[videoName]]}`);
    // console.log(`watcher ${videoSelect.options[videoSelect.selectedIndex].text}`);
    // socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);
    const peerConnection = new RTCPeerConnection(config);
    peerConnections[id] = peerConnection;

    // let stream = watcherVideo.srcObject;
    const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });

    stream.getTracks().forEach((track) => {peerConnection.addTrack(track, stream)});
    console.log(stream);
    // peerConnection.addStream(stream);

    peerConnection.onicecandidate = event => {
      if (event.candidate) {
        socket.emit("candidate", id, event.candidate);
      }
    };

    console.log("Set local descp")
    console.log(`Sending offer to ${id}`)
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

  socket.on("answer", (id, description) => {
    peerConnections[id].setRemoteDescription(description);
  });

  socket.on("offer", (id, description) => {
    console.log("Got offer");
    peerConnection = new RTCPeerConnection(config);
    peerConnections[id] = peerConnection;
    peerConnection
      .setRemoteDescription(description)
      .then(() => peerConnection.createAnswer())
      .then(sdp => peerConnection.setLocalDescription(sdp))
      .then(() => {
        socket.emit("answer", id, peerConnection.localDescription);
      });
    peerConnection.ontrack = event => {
      broadcasterVideo.srcObject = event.streams[0];
      console.log(event.streams[0]);
    };
    peerConnection.onicecandidate = event => {
      if (event.candidate) {
        socket.emit("candidate", id, event.candidate);
      }
    };
  });


  socket.on("candidate", (id, candidate) => {
    console.log(id);
    peerConnections[id]
      .addIceCandidate(new RTCIceCandidate(candidate))
      .catch(e => console.error(e));
  });

  socket.on("connect", () => {
    socket.emit("watcher",socket.id);
    socket.emit("broadcaster");
  });

  socket.on("broadcaster", () => {
    socket.emit("watcher",socket.id);
  });

  socket.on("disconnectPeer", (id) => {
    peerConnections[id].close();
    delete peerConnections[id];
  });
  if (mobileDevice) {
    // socket.emit("switchCam", viewSwitch.checked);
  } else {
  }
  socket.on("switchedCam", (videoSelect) => {


    // get the index of the camera 0=3rd-person, 1=1st-person
    externalCam = Boolean(camList.indexOf(videoSelect));
    // if mobile, show 1st-person if external cam
    // if (mobileDevice) {
    //   video.width = externalCam ? window.innerWidth : 0;
    // // if desktop, show 3rd-person if not external cam
    // } else {
    //   video.width = externalCam ? 0 : window.innerWidth*0.5;
    // };
  });  
  socket.on("updateVideo", (videoName) => {
    curVideo = videoName;
  });
  socket.on("getVideo", () => {
    if (curVideo.length>1) {
      socket.emit("updateVideo", curVideo);
    };
  });

  socket.on("promptText", (promptText) => {
    promptSpace.innerHTML = promptText.replace('A',' a');
    promptSpace.innerText = promptText.replace('A',' a');
  });
  socket.on("newVideo", (videoName) => {
    console.log(curVideo);
    curVideo = videoName;
    ytVid.src = `https://www.youtube.com/embed/${ytDict[curVideo]}?autoplay=1&modestbranding=1&rel=1`

    // show video if on desktop
    if (!mobileDevice) {
      ytVid.width = window.innerWidth/2;
      ytVid.height = ytVid.width*3/4;
    }
  });
  socket.on("hideVideo", () => {
    // hide video if on desktop
    if (!mobileDevice) {
      ytVid.width = 0;
      ytVid.height = 0;
    }
  });
  // hide survey if on mobile
  if (mobileDevice) { 
    survey.style.display = "none";
  } else {
    // show survey
    socket.on("showSurvey", (showSurvey) => {
      survey.hidden = !showSurvey;
      survey.width = showSurvey ? window.innerWidth : 0;
      survey.height = showSurvey ? 1000 : 0;
    });
  }
  socket.emit("getVideo");

}

function updateVideo() {
  // don"t get value if empty field
  if (videoId.value.length<2) { return; };
  curId = videoId.value.trim();

  curVideo = ytIdDict[curId];
  ytVid.src = `https://www.youtube.com/embed/${curId}?autoplay=1&modestbranding=1&rel=1`

  console.log(curId, curVideo);
  socket.emit("newVideo", curVideo);

  if (!mobileDevice) {
    ytVid.width = window.innerWidth/2;
    ytVid.height = ytVid.width*3/4;
  }
}

function toggleView() {
  // switch view
  socket.emit("switchCam", viewSwitch.checked);

  // disable the video in 3rd-person mode to force viewing on desktop
  // video.width = (mobileDevice && viewSwitch.checked) ? 0 : window.innerWidth;
}

function compassHeading( alpha, beta, gamma ) {
  var degtorad = Math.PI / 180; // Degree-to-Radian conversion

  var _x = beta  ? beta  * degtorad : 0; // beta value
  var _y = gamma ? gamma * degtorad : 0; // gamma value
  var _z = alpha ? alpha * degtorad : 0; // alpha value

  var cX = Math.cos( _x );
  var cY = Math.cos( _y );
  var cZ = Math.cos( _z );
  var sX = Math.sin( _x );
  var sY = Math.sin( _y );
  var sZ = Math.sin( _z );

  // Calculate Vx and Vy components
  var Vx = - cZ * sY - sZ * sX * cY;
  var Vy = - sZ * sY + cZ * sX * cY;

  // Calculate compass heading
  var compassHeading = Math.atan( Vx / Vy );

  // Convert compass heading to use whole unit circle
  if( Vy < 0 ) {
    compassHeading += Math.PI;
  } else if( Vx < 0 ) {
    compassHeading += 2 * Math.PI;
  }
  // return compassHeading;
  return compassHeading * ( 180 / Math.PI ); // Compass Heading (in degrees)

}


const handleOrientation = ( e) => {
  if (!controlSwitch.checked && !appendageSwitch.checked) {
    return;
  }

  beta = e.beta*Math.PI/180;
  gamma = e.gamma*Math.PI/180;

  alphaTrue = compassHeading(e.alpha, e.beta, e.gamma);
  alpha = (alphaTrue-yawOffset)*Math.PI/180;

  // constraint 0 < alpha < 2pi
  if (alpha < 0) {alpha +=2*Math.PI;}
  if (alpha > 2*Math.PI) {alpha-=2*Math.PI;}

  var endpointThreshold = Math.PI/6;
  var distToEndpoint = Math.abs(Math.PI-alpha);
  // beta<2 because sometimes indicates when just looking up
  // doesnt catch high beta in the endpoints anyways
  // only catches if pitch is not much higher than the horizon
  if (!inEndpoint && distToEndpoint<endpointThreshold && beta<2) {
    inEndpoint = true;
    if (Math.PI>alpha) {
      endpointIndicator.style.textAlign="left";
      endpointIndicator.style.left="10%";
      endpointIndicator.innerHTML = "<";
    } else {
      endpointIndicator.style.textAlign="right";
      endpointIndicator.style.left="90%";
      endpointIndicator.innerHTML = ">";
    }
    if (!viewSwitch.checked && showEndpointAlerts) {
      // showEndpointAlerts = confirm("Hit endpoint, turn around!\nPress Cancel to block future popups.");
    }
  // increase endpoint to make it harder to get out of it
  // this should reduce the frequency of alerts
  } else if (inEndpoint && distToEndpoint>2*endpointThreshold) {
    inEndpoint = false;
    endpointIndicator.innerHTML = " ";
  }

  if (e.timeStamp-lastEvent > 50) {
    lastEvent = e.timeStamp;
    var body = {
        x: beta,
        y: gamma,
        z: alpha, 
        ax: 0,
        ay: 0,
        az: 0,
        h: defaultHeight,
        h: parseInt(heightPos),
        q: 0,
        ears: 50,
        left_arm: -leftArmPos,
        right_arm: -rightArmPos,
        tail: tailPos,
        landscape: false,
        // mirror: !externalCam,
        // mirror: true,
        mirror: false,
        heightCtrl: false,
        armsCtrl: appendageSwitch.checked,
        headCtrl: controlSwitch.checked,
        yaw: 0,
        time: Date.now(),
        portrait: true,
      };

    ctrlSocket.emit("position", body)
  } 
}

function record() {
  // user must have given control consent first
  if (!controlConsent) {
    alert("Turn control on first!");
    return;
  }
  var _cmd = recordMode;
  var gestureName = "";
  if (recordMode == "stop") {
    // turn off controlling if not already
    if (controlSwitch.checked) {
      controlSwitch.click();
    }
    if (userNamesGesture) {
      while (gestureList.includes(gestureName) || gestureName.length == 0) {
        if (gestureList.includes(gestureName)) {
          msg = "Name already taken, please choose another: "
        };
        if (gestureName.length == 0) {
          msg = "Please enter a descriptive name for your gesture: "
        };
        gestureName = window.prompt(msg).replace(/ /g,"_");
        if (gestureName == null) {
          gestureName = "null";
          break;
        }
      }
    } else {

     if (confirm("Save the current movement?")) {
      var curDate = new Date();
      gestureName = `${externalCam ? "fps" : "tps"}:${curVideo}_`
        +`${curDate.getFullYear()}${curDate.getMonth()+1}${curDate.getDate()}_`
        +`${curDate.getHours()}${curDate.getMinutes()}${curDate.getSeconds()}`;
      } else {
        gestureName = "null";
      }
    }
    _cmd += "/"+gestureName;
  }

  fetch(ctrlAddr+`/record/`+_cmd, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
    })
    .then(() => {
      if (recordMode == "start") {
        // turn on controlling if not already on
        if (!controlSwitch.checked) {
          controlSwitch.click();
        }

        // change button function
        recordMode = "stop";
        recordButton.classList.remove("notRec");
        recordButton.classList.add("Rec");
      } else {
        recordMode = "start";
        recordButton.classList.remove("Rec");
        recordButton.classList.add("notRec");
        getGestures();
      }
    })
    .catch(() => {
      console.log("Couldnt send to bot")
    });
}

function calibrateYaw() {
  heightSlider.value = "70";
  fetch(ctrlAddr+`/reset`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
    })
    .then(() => {
      yawOffset = alphaTrue;
    })
    .catch(() => {
      console.log("Couldnt calibrate yaw")
    });
}

function getGestures() {
  fetch(ctrlAddr+`/sequences`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
    },
    // mode: "no-cors",
  })
  .then (response => response.json())
  .then( (list) => {
    // empty the current list of gestures
    document.querySelectorAll("#gestures option").forEach(o => o.remove());
    list.unshift("reset");

    if (!userNamesGesture) {
      list = ["happy", "sad", "anger"];
    }

    gestureList = [];

    for (var i=0; i < list.length; i++) {
      var gestureName = list[i];

      if (gestureName.includes("/") || gestureList.includes(gestureName)) {
        console.log("Skipping");
        continue;
      }
      gestureList.push(gestureName);

      var opt = document.createElement("option");
      opt.value = gestureName;
      opt.innerHTML = gestureName;
      if (gestureList.includes(opt)) {
        console.log("Already in list, skipping");
      }

      gestures.appendChild(opt);
    }
    console.log(sequenceList);
  })
  .catch(() => {
      console.log("Couldnt get gesture list")
  });

}

function playGesture() {
  if (controlSwitch.checked) {
    controlSwitch.click();
  }
  fetch(ctrlAddr+`/s/`+document.getElementById("gestures").value, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
    },
  })
  .catch(() => {
      console.log("Couldnt play gesture")
  });
}


function onControl() {
  controlConsent = true;
  if (controlSwitch.checked || appendageSwitch.checked) {
    if (controlSwitch.checked && calibrateEveryControl) { calibrateYaw(); };
    // feature detect
    if (iosDevice) {
      DeviceMotionEvent.requestPermission()
        .then( permissionState => {
          if ( permissionState === "granted" ) {
            window.addEventListener(orientationEvent, handleOrientation, false);
          }
        } )
        .catch( console.error );
    } else {
      if (window.DeviceMotionEvent) {
          window.addEventListener(orientationEvent, handleOrientation, false);
      }
    }
  } else {
    window.removeEventListener(orientationEvent, handleOrientation);
  }
}

const config = {
  iceServers: [
    {  urls: [ "stun:us-turn6.xirsys.com" ]},
    {   username: "uTZru2n264RQqZAwDgHGuAt6tj8GV5cRJGQDrYMi65Hbogw1JEcFjVNnk9W6DzVIAAAAAF-Dv9Vwc3ljaG9tdWdz",  
      credential: "ee851dfc-0c32-11eb-a231-0242ac140004",   
      urls: [       "turn:us-turn6.xirsys.com:80?transport=udp",       
      "turn:us-turn6.xirsys.com:3478?transport=udp",       
      "turn:us-turn6.xirsys.com:80?transport=tcp",       
      "turn:us-turn6.xirsys.com:3478?transport=tcp",       
      "turns:us-turn6.xirsys.com:443?transport=tcp",       
      "turns:us-turn6.xirsys.com:5349?transport=tcp"   
      ]
    }
  ]
};


window.onunload = window.onbeforeunload = () => {
  socket.close();
};


window.onresize = () => {
  if (broadcasterVideo.width != 0) {
    broadcasterVideo.width = window.innerWidth * (mobileDevice ? 1 : 0.5);
    broadcasterVideo.height = broadcasterVideo.width*3/4;
    broadcasterVideo.width = window.innerHeight * (mobileDevice ? 1.5 : 0.5);
    broadcasterVideo.height = broadcasterVideo.width*3/4;
  }
  if (!mobileDevice) {
    if (ytVid.width != 0) {
      ytVid.width = window.innerWidth/2;
      ytVid.height = ytVid.width*3/4;
    }
  }
};


audioSelect.onchange = getStream;
videoSelect.onchange = getStream;

getStream(videoSelect)
  .then(getDevices)
  .then(gotDevices);

checkPswd();


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
    // video: { deviceId: videoSource ? { exact: videoSource } : undefined }
  };
  return navigator.mediaDevices
    .getUserMedia(constraints)
    .then(gotStream)
    .catch(handleError);
  }

function gotStream(stream) {
  window.stream = stream;
  audioSelect.selectedIndex = [...audioSelect.options].findIndex(
    option => option.text === stream.getAudioTracks()[0].label
  );
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    option => option.text === stream.getVideoTracks()[0].label
  );
  watcherVideo.srcObject = stream;
  socket.emit("broadcaster");
  // console.log(`gotStream ${videoSelect.options[videoSelect.selectedIndex].text}`);
  socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);
}

function handleError(error) {
  console.error("Error: ", error);
}