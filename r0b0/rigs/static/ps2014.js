var socketAddr = "r0b0.ngrok.io";
console.log(socketAddr);
socket = io.connect(window.location.href);

var closeButton = document.getElementById('closeButton');
var openButton = document.getElementById('openButton');

let buttons = [closeButton,openButton];
for (let i=0; i<buttons.length; i++) {
    buttons[i].style.width=window.innerWidth;
    buttons[i].style.height=window.innerHeight/buttons.length;
}

closeButton.onclick = function() {
    console.log('closeButton');
    socket.emit('button',
        {
            event:'button',
            value:6000,
            id:socket.id
        });
}

openButton.onclick = function() {
    console.log('openButton');
    socket.emit('button',
        {
            event:'button',
            value:6000-270000,
            id:socket.id,
        });
}