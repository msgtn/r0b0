const socketAddr = "https://r0b0.ngrok.io";

const socket = io.connect(socketAddr)
const tapesSelect = document.getElementById("tapes");
const loop = document.getElementById("loop");

function updateTapes() {
  fetch(`${socketAddr}/tapes`)
    .then(response =>  response.json())
    .then(data => {
      console.log(tapesSelect);
      console.log(data);
      data.map((lang, i) => {
        let opt = document.createElement("option");
        opt.value = i; // the index
        opt.innerHTML = lang.replace(".json","");
        console.log(opt);
        // tapesSelect.append(opt);
        tapesSelect[i] = opt;
      })
    })
  ;
}

function playTape() {
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
  socket.emit("stop",
    {
      "tape_name": selectedTape(),
    }
  )
}

updateTapes();