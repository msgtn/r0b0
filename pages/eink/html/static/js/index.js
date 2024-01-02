const socketAddr = "https://r0b0.ngrok.io";
// const socketAddr = "https://localhost:8080";
var socket = io.connect(socketAddr, {
    maxHttpBufferSize: 1e8
});
var contents = null;
var fileinput = document.getElementById('file');

window.addEventListener('submit', handleUpload, false);

function handleUpload(e) {
    /**
     * Handle file uploads
     */
    e.preventDefault();

    // Get the first (only) input file
    var fileInput = document.getElementById('file');
    var file = fileInput.files[0];

    if (file) {
        console.log("Received file");
        var reader = new FileReader();

        reader.readAsArrayBuffer(file);
        reader.onload = function (e) {

            contents = e.target.result;
            console.log('File contents:', contents);

            // Load the images
            var blob = new Blob([contents]);
            var blobURL = window.URL.createObjectURL(blob);
            var image = new Image();
            image.src = blobURL;
            image.onload = function () {
                // Resize the image so it can pass through the <1MB socket limit
                var resized = resizeMe(image);
                var resizedBuffer = dataURLToArrayBuffer(resized);

                // Emit the image 
                var body = {
                    event: "file_upload",
                    image: new Uint8Array(resizedBuffer),
                    time: Date.now(),
                    id: socket.id,
                };
                socket.emit("file_upload", body);
                console.log(body);

            }
        };

    }
};

// Copied from https://github.com/josefrichter/resize/blob/master/public/preprocess.js
function resizeMe(img) {
    /**
     * Resize an image for lightweight passing through the socket
     */

    var canvas = document.createElement('canvas');

    canvas.width = 264;
    canvas.height = 176;
    var ctx = canvas.getContext("2d");
    ctx.drawImage(img, 0, 0,
        // img.width, img.height);
        canvas.width, canvas.height);   // Resize to the size of the e-ink display

    return canvas.toDataURL("image/jpeg", 0.1); // get the data from canvas as 70% JPG (can be also PNG, etc.)

}

function dataURLToArrayBuffer(dataURL) {
    /**
     * Helper function to convert the image from a data URL to an array buffer for emitting
     */
    // Remove the data URL prefix
    const base64String = dataURL.split(',')[1];

    // Decode the base64 string
    const binaryString = atob(base64String);

    // Create a Uint8Array from the binary string
    const length = binaryString.length;
    const arrayBuffer = new ArrayBuffer(length);
    const uint8Array = new Uint8Array(arrayBuffer);
    for (let i = 0; i < length; i++) {
        uint8Array[i] = binaryString.charCodeAt(i);
    }

    return arrayBuffer;
}