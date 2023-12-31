export function getDevices() {
  return navigator.mediaDevices.enumerateDevices();
}

export function gotDevices(deviceInfos, videoSelect) {
  window.deviceInfos = deviceInfos;
  for (const deviceInfo of deviceInfos.reverse()) {
    const option = document.createElement("option");
    option.value = deviceInfo.deviceId;
    if (deviceInfo.kind === "audioinput") {
      // option.text = deviceInfo.label || `Microphone ${audioSelect.length + 1}`;
      // audioSelect.appendChild(option);
    } else if (deviceInfo.kind === "videoinput") {
      option.text = deviceInfo.label || `Camera ${videoSelect.length + 1}`;
      var videoOptions = [...videoSelect.options].map(o => o.label);
      if (!videoOptions.includes(deviceInfo.label)) {
        videoSelect.appendChild(option);
      };
    }
  }
}

export function getStream(videoSelect) {
  if (window.stream) {
    window.stream.getTracks().forEach(track => {
      track.stop();
    });
  }
  // const audioSource = audioSelect.value;
  const videoSource = videoSelect.value;
  const constraints = {
    // audio: { deviceId: audioSource ? { exact: audioSource } : undefined },
    video: { deviceId: videoSource ? { exact: videoSource } : undefined }
  };
  return navigator.mediaDevices
    .getUserMedia(constraints)
    .then(gotStream)
    .catch(handleError);
}

export function gotStream(stream, videoSelect) {
  window.stream = stream;
  // audioSelect.selectedIndex = [...audioSelect.options].findIndex(
  //   option => option.text === stream.getAudioTracks()[0].label
  // );
  videoSelect.selectedIndex = [...videoSelect.options].findIndex(
    option => option.text === stream.getVideoTracks()[0].label
  );
  videoElement.srcObject = stream;
  socket.emit("broadcaster");
  console.log(`gotStream ${videoSelect.options[videoSelect.selectedIndex].text}`);
  socket.emit("switchedCam", videoSelect.options[videoSelect.selectedIndex].text);
}

export function handleError(error) {
  console.error("Error: ", error);
}