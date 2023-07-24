const socketAddr = "https://r0b0.ngrok.io";

const socket = io.connect(socketAddr)

function updateTapes() {
  let tapesSelect = document.getElementById("tapes");
  fetch(`${socketAddr}/tapes`)
    .then(response =>  response.json())
    .then(data => {
      data.map((lang, i) => {
        let opt = document.createElement("option");
        opt.value = i; // the index
        opt.innerHTML = lang.replace(".json","");
        // tapesSelect.append(opt);
        tapesSelect[i] = opt;
      })
    })
  ;
}

function playTape() {
  let tapesSelect = document.getElementById("tapes");
  let selectedTape = tapesSelect.options[tapesSelect.selectedIndex].text;
  socket.emit("play",
    {'tape_name':selectedTape}
  )
}

updateTapes();