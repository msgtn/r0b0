
const socketAddr = "https://r0b0.ngrok.io";
var socket = io.connect(socketAddr);
var contents = null;

var form = document.getElementById('form')
window.addEventListener('submit', submitForm, false);

function submitForm(e) {
    e.preventDefault();
    
    var fileInput = document.getElementById('file');
    var file = fileInput.files[0];
    
    if (file) {
        console.log("Received file");
        var reader = new FileReader();
        // reader.readAsBinaryString(file);
        
        reader.onload = function(e) {

            contents = e.target.result;
            // contents = contents.replace(/.*base64,/, '');
            console.log('File contents:', contents);
            var body = {
                event: "file_upload",
                // image: btoa(contents),
                // image: contents,
                image: new Uint8Array(contents),
                time: Date.now(),
                id: socket.id,
                // namespace: "/eink_page",
            };


            socket.emit("file_upload", body);
            console.log(body);
            // Perform further processing with the file contents
        };
        
        // reader.readAsText(file);
        // reader.readAsDataURL(file);
        // reader.readAsBinaryString(file);
        reader.readAsArrayBuffer(file);
    }
};
