
const peerConnections = {};


const videoNameSelect = document.getElementById('videoName');
const emotions = ['happy', 'sad', 'anger'];
var vidViewDict = {}
function generateVidView() {
  var vidNames = [];
  // subtract 2 for the test Elmo videos
  for (var i=0; i<videoNameSelect.options.length-2; i++) {
    vidNames.push(videoNameSelect.options[i].value);
  }
  console.log(vidNames);
  console.log(Math.ceil(vidNames.length/6));
  // array of probabilities
  var emProb = Array(Math.ceil(vidNames.length/6)).fill([0,1]).flat();
  var emotionProbs = []
  for (i=0; i<emotions.length; i++) {
    emotionProbs.push([...emProb]);
  }

  vidNames.forEach(function (vidName, index) {
    var vidEmotionIdx = emotions.indexOf(vidName.split('_')[0]);
    // randomly select video view
    var vidView = emotionProbs[vidEmotionIdx].splice(Math.floor(Math.random()*emotionProbs[vidEmotionIdx].length), 1)[0];
    vidViewDict[vidName] = vidView;
    console.log(emotionProbs);
  })

  // tutorial vids
  vidViewDict['happy_elmo'] = emProb[Math.floor(Math.random()*emProb.length)];
  vidViewDict['sad_elmo'] = Boolean(vidViewDict['happy_elmo']) ? 0 : 1;
}
generateVidView();

// override vidViewDict here if necessary
// vidViewDict = {"happy_pika":0,"sad_pika":0,"anger_pika":1,"happy_sponge":0,"sad_sponge":1,"anger_sponge":0,"happy_homer":1,"sad_homer":0,"anger_homer":1,"happy_elmo":1,"sad_elmo":0};
console.log(vidViewDict)

const config = {
  iceServers: [
    {   urls: [ "stun:us-turn6.xirsys.com" ]},
    {   username: "uTZru2n264RQqZAwDgHGuAt6tj8GV5cRJGQDrYMi65Hbogw1JEcFjVNnk9W6DzVIAAAAAF-Dv9Vwc3ljaG9tdWdz",   credential: "ee851dfc-0c32-11eb-a231-0242ac140004",   urls: [       "turn:us-turn6.xirsys.com:80?transport=udp",       "turn:us-turn6.xirsys.com:3478?transport=udp",       "turn:us-turn6.xirsys.com:80?transport=tcp",       "turn:us-turn6.xirsys.com:3478?transport=tcp",       "turns:us-turn6.xirsys.com:443?transport=tcp",       "turns:us-turn6.xirsys.com:5349?transport=tcp"   ]}]
};

var newVideoButton = document.getElementById("newVideo");
var hideVideoButton = document.getElementById("hideVideo");
var showSurveyButton = document.getElementById("showSurvey");
var hideSurveyButton = document.getElementById("hideSurvey");
var saveVidViewDictButton = document.getElementById("saveVidViewDict");
var promptSelect = document.getElementById("promptSelect");
var embodimentSelect = document.getElementById("embodimentSelect");
var webmCheckbox = document.getElementById("webmCheckbox");
videoNameSelect.style.display = "none";
newVideoButton.style.display = "none";
hideVideoButton.style.display = "none";
// showSurveyButton.style.display = "none";
// hideSurveyButton.style.display = "none";
saveVidViewDictButton.style.display = "none";

// Get camera and microphone
const broadcasterVideo = document.getElementById("broadcasterVideo");
var watcherVideo = document.getElementById("watcherVideo");
var promptSpace = document.getElementById("promptSpace");
var idSpace = document.getElementById("idSpace");
broadcasterVideo.width = watcherVideo.width = parseInt(window.innerWidth/2);
broadcasterVideo.height = watcherVideo.height = broadcasterVideo.width*3/4;
const audioSelect = document.getElementById('audioSource');
const videoSelect = document.getElementById('videoSource');
var videoName = 'happy_elmo';
var curPromptKey = ''

// 0=laptop, 1=external

const socket = io.connect(window.location.origin);
console.log(socket.id);

socket.on("connect", () => {
  console.log("Connecting");
  socket.emit("watcher",socket.id);
  socket.emit("broadcaster");
  socket.emit("promptText",defaultPrompt);
});

function sendWatcher() {
  socket.emit("watcher",socket.id);
  setTimeout(function() {
    socket.emit("broadcaster");
  }, 1000);
};

var curID = "";

function generateID(length=5) {
    var result           = '';
    var characters       = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    var charactersLength = characters.length;
    for ( var i = 0; i < length; i++ ) {
      result += characters.charAt(Math.floor(Math.random() * charactersLength));
    }
   // return result;
   curID = result;
   idSpace.innerHTML = curID;
}
generateID();

var promptKeys = Object.keys(promptDict);
for (var i=0; i<promptKeys.length; i++) {
  var el = document.createElement("option");
  el.textContent = promptKeys[i];
  el.value = promptKeys[i];
  promptSelect.appendChild(el);
}

function sendEmbodiment() {
  socket.emit('embodimentText',embodimentSelect.value);
}


function showPrompt() {
  // curPrompt = promptDict[promptSelect.value];
  // let embodimentText = embodimentSelect.options[embodimentSelect.selectedIndex].text;
  promptSpace.innerHTML = `${embodimentSelect.options[embodimentSelect.selectedIndex].text}: ${promptDict[promptSelect.value]}`;
  // promptSpace.innerHTML = curPrompt : 'Prompt text will appear here.';
  socket.emit('showPrompt', $('#promptCheckbox').prop('checked') ? promptSpace.innerHTML : 'Prompt text will appear here.')
}
var videoWebm = document.getElementById('videoWebm');
videoWebm.style.display="none";

function showWebm() {
  videoWebm.width = $('#webmCheckbox').prop('checked') ? broadcasterVideo.width : 0;
  videoWebm.height = videoWebm.width*3/4; 
  // playWebmButton.style.display = showWebm ? "block" : "none";
  videoWebm.load();
  socket.emit('showWebm', $('#webmCheckbox').prop('checked'));
}
// socket.emit("watcher");
// socket.emit("broadcaster");


function playWebm() {
  videoWebm.currentTime = 0;

  playMostRecent();
  setTimeout(function() {
    videoWebm.play();
  },600);

}

socket.on("answer", (id, description) => {
  peerConnections[id].setRemoteDescription(description);
});

socket.on("broadcaster", () => {
  socket.emit("watcher",socket.id);
});

socket.on("watcher", async (id) => {
  // console.log(`watcher ${camList[vidViewDict[videoName]]}`);
  console.log(`watcher ${videoSelect.options[videoSelect.selectedIndex].text}`);
  socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);
  const peerConnection = new RTCPeerConnection(config);
  peerConnections[id] = peerConnection;

  const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });

  stream.getTracks().forEach((track) => {peerConnection.addTrack(track, stream)});

  peerConnection.onicecandidate = event => {
    if (event.candidate) {
      socket.emit("candidate", id, event.candidate);
    }
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

// commanded by user that a new video is being used
socket.on("newVideo", (videoName) => {
  socket.emit("updateVideo", videoName);
  document.getElementById('videoName').value = videoName;
  // update the perspective based on the chosen video
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    option => option.text === vidViewDict[videoName]
  );
  videoSelect.options[vidViewDict[videoName]].selected = true;
  getStream(videoSelect)
    .then(getDevices)
    .then(gotDevices)
})

window.onunload = window.onbeforeunload = () => {
  socket.close();
};

socket.on("getVidViewDict", () => {
  console.log("getVidViewDict");
  socket.emit("sentVidViewDict", vidViewDict);
});

audioSelect.onchange = getStream;
videoSelect.onchange = getStream;

getStream(videoSelect)
  .then(getDevices)
  .then(gotDevices);

socket.on("playWebm", () => { playMostRecent(); });


function newVideo() {
  socket.emit("newVideo", document.getElementById('videoName').value);
}

function hideVideo() {
  socket.emit("hideVideo");
}

function showSurvey() {
  socket.emit("showSurvey", $('#surveyCheckbox').prop('checked'));
  saveVidViewDict();
}



function saveVidViewDict() {
  socket.emit("saveVidViewDict", vidViewDict);
}


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
  socket.emit("broadcaster");
  // socket.emit("watcher");
  console.log("Got stream");
  console.log(`gotStream ${videoSelect.options[videoSelect.selectedIndex].text}`);
  socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);

}



function handleError(error) {
  console.error("Error: ", error);
}

var mediaRecorder = null;
var recordedChunks = [];
var recordButton = document.getElementById("recordButton");
var firstRec = true;
function startRecord() {
  console.log("Start recording");
  var options = { mimeType: "video/webm; codecs=vp9" };
  mediaRecorder = new MediaRecorder(watcherVideo.srcObject, options);
  mediaRecorder.ondataavailable = function(event) {
    console.log("data-available");
    if (event.data.size>0) {
      recordedChunks.push(event.data);
      console.log(recordedChunks);
      // stopRecord();
      if (firstRec) {
        // a.click();
        stopRecord();
        // firstRec = false;
      }
    } else {
      // ...
    };
  };
  for (let i = 0; i<=3; i++) {
    setTimeout(function() {
      console.log(i);
      if (i<3) {
        let i_str = (3-i).toString();
        recordButton.value = i_str;
        socket.emit("updateRecInd", i_str);
      } else {
        mediaRecorder.start();
        recordButton.value = "Recording";
        recordButton.onclick = stopRecord;
        socket.emit("updateRecInd", "R");
      }
    }, 1000*i);
  };
};

var recCtr = 0
function stopRecord() {
  let date = new Date();
  console.log("Stopping");
  console.log(recordedChunks.length);
  // if (recordedChunks.length==0) { return; };

  var blob = new Blob(recordedChunks, {
    type: "video/webm"
  });
  var url = URL.createObjectURL(blob);
  var a = document.createElement("a");
  document.body.appendChild(a);
  a.style = "display: none";
  a.href = url;
  var dateStr = date.toISOString()
  dateStr = dateStr.replace(/:\s*/g,'-').replace('.','-').replace('_','-');
  console.log(dateStr);
  // a.download = `vp_${dateStr}_${curID}_${embodimentSelect.value.replace(' ','_')}_${promptSelect.value}.webm`;
  a.download = `vp_${dateStr}_${promptSelect.value}_${embodimentSelect.value.replace(' ','_')}_${curID}.webm`;
  // if (recordedChunks.length>0) { a.click(); };
  if (recCtr==0) {
    recCtr++;
  } else {
   a.click();
   recCtr = 0;
  }
  // if (firstRec) {
  //   a.click();
  //   firstRec = false;
  // }
  window.URL.revokeObjectURL(url);
  mediaRecorder.stop();

  // arm to record again
  recordButton.value = "Start Record";
  recordButton.onclick = startRecord;
  socket.emit("updateRecInd", "");

  recordedChunks = [];
}