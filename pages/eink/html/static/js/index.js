// import { Server } from "socket.io";
// import require;
// const { Server } = require("socket.io");
// import SocketIO from "socket.io";
// import {Server, Socket} from 'socket.io';
// var http = require('http');
// import { createServer } from 'http';
// import * as http from './node_modules/http';
// import createServer from http;
// var http = require('http');

// import * as io from 'https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js';
// import {Server} from 'https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.2/socket.io.js';

const socketAddr = "https://r0b0.ngrok.io";
// const httpServer = http.createServer();
// io = new Server(socketAddr, {
//     maxHttpBufferSize: 1e8
// });
var socket = io.connect(socketAddr, {
    maxHttpBufferSize: 1e8
});
var contents = null;
var fileinput = document.getElementById('file');

var max_width = fileinput.getAttribute('data-maxwidth');
var max_height = fileinput.getAttribute('data-maxheight');

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
            var blob = new Blob([contents]);
            var blobURL = window.URL.createObjectURL(blob);
            var image = new Image();
            image.src = blobURL;
            image.onload = function() {
                var resized = resizeMe(image);
                var resizedBuffer = dataURLToArrayBuffer(resized);
                // var newinput = document.createElement("input");
                // newinput.src = resized;
                // // newinput.type = 'hidden';
                // // newinput.name = 'images[]';
                // // newinput.value = resized; // put result from canvas into new hidden input
                // // form.appendChild(newinput);
                // // console.log(resized)
                // var resized64string = resized.split(',')[1];
                // var binaryString = atob(resized64string);
                // var resizedArray = new Uint8Array(atob(resized.split(',')[1]));
                // var length = binaryString.length;
                // const arrayBuffer = new ArrayBuffer(length);
                // var resizedArray = new Uint8Array(arrayBuffer);
                // for (let i = 0; i < length; i++) {
                //     uint8Array[i] = binaryString.charCodeAt(i);
                //   }
                // console.log(resizedArray)
                var body = {
                    event: "file_upload",
                    // image: btoa(contents),
                    // image: contents,
                    // image: new Uint8Array(contents),
                    image : new Uint8Array(resizedBuffer),
                    // image: new Uint8Array(atob(newinput)),
                    // image: new Uint8Array(resized, c => c.charCodeAt(0)),
                    // image: resized,
                    time: Date.now(),
                    id: socket.id,
                    // namespace: "/eink_page",
                };


                socket.emit("file_upload", body);
                console.log(body);

            }
            // Perform further processing with the file contents
        };
        
        // reader.readAsText(file);
        // reader.readAsDataURL(file);
        // reader.readAsBinaryString(file);
        reader.readAsArrayBuffer(file);
    }
};

// Copied from https://github.com/josefrichter/resize/blob/master/public/preprocess.js
function resizeMe(img) {
  
    var canvas = document.createElement('canvas');
  
    // var width = img.width;
    // var height = img.height;
    // var width = 264;
    // var height = 176;
  
    // // calculate the width and height, constraining the proportions
    // if (width > height) {
    //   if (width > max_width) {
    //     //height *= max_width / width;
    //     height = Math.round(height *= max_width / width);
    //     width = max_width;
    //   }
    // } else {
    //   if (height > max_height) {
    //     //width *= max_height / height;
    //     width = Math.round(width *= max_height / height);
    //     height = max_height;
    //   }
    // }
    
    // resize the canvas and draw the image data into it
    canvas.width = img.width;
    canvas.height = img.height;
    var ctx = canvas.getContext("2d");
    ctx.drawImage(img, 0, 0, img.width, img.height);
    
    // preview.appendChild(canvas); // do the actual resized preview
    
    return canvas.toDataURL("image/jpeg",0.2); // get the data from canvas as 70% JPG (can be also PNG, etc.)
    // return canvas.("image/jpeg",0.7); // get the data from canvas as 70% JPG (can be also PNG, etc.)
  
  }function dataURLToArrayBuffer(dataURL) {
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