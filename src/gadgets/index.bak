// 'use strict'

const port = 8000
// const port = 443
const getIp = require('internal-ip').v4
// const server = require('http-server').createServer()
const https = require('https');
const http = require('http');
const fs = require('fs');
var express = require('express');
var app = express();
const htpasswd = require('htpasswd-js');
const basicAuth = require('express-basic-auth');
var passport = require('passport');
var LocalStrategy = require('passport-local').Strategy;
var passwords = JSON.parse(fs.readFileSync('./passwords.json','utf8'))[0];
console.log(passwords);
for (pw in passwords) {
  console.log(`blossombot.com username:${pw} password:${passwords[pw]}`);
}

process.on('SIGINT', function() {
  console.log("Interrupted");
  server.close();
  ctrlServer.close();
  process.exit();
})

const options = {
    key: fs.readFileSync('./key.pem'),
    cert: fs.readFileSync('./cert.pem')
}

app.use(basicAuth({
  users: passwords,
  challenge: true,
}))

app.use(express.static('./public'))


var ctrl_addr = "";
fs.readFile('public/ctrl_addr.txt', (err,data) => {
  if (err) throw err;
  console.log(data.toString());
  ctrl_addr = data.toString();
})


if (app.get('env') ==='development'){
    var server = https.createServer(options, app);
    var ctrlServer = https.createServer(options, app);


    const io = require("socket.io")(server, {origins: '*:*'});
    const ctrlIO = require('socket.io')(ctrlServer);
    app.use(express.static(__dirname + "/public"));

    var connections = []

    io.sockets.on("error", e => console.log(e));
    io.sockets.on("connection", socket => {
      socket.on("broadcaster", () => {
        broadcaster = socket.id;
        socket.broadcast.emit("broadcaster");
        // socket.broadcast.emit("watcher");
        socket.broadcast.emit("getVidViewDict");
      });
      socket.on("watcher", (id) => {
        if (typeof broadcaster !== 'undefined') {
          console.log(id);
          console.log(socket.id);
          // socket.emit("watcher",socket.id);
          socket.to(broadcaster).emit("watcher", socket.id);
          // socket.to(id).emit("watcher", socket.id);
          // socket.to(socket.id).emit("watcher", id);
        }
      });
      socket.on("offer", (id, message) => {
        socket.to(id).emit("offer", socket.id, message);
      });
      socket.on("answer", (id, message) => {
        connections.push(socket.id);
        // console.log(connections);
        if (connections.length<100) {
          socket.to(id).emit("answer", socket.id, message);
          // console.log("Connection from", socket.id);
        }
      });
      socket.on("candidate", (id, message) => {
        socket.to(id).emit("candidate", socket.id, message);
      });
      socket.on("disconnect", () => {
        if (typeof broadcaster !== 'undefined') {
          socket.to(broadcaster).emit("disconnectPeer", socket.id);
          connections.splice(connections.indexOf(socket.id),1);
          // console.log(connections);
          // console.log("Disconnection from", socket.id);
        }
      });
      socket.on("updateRecInd", (text) => {
        socket.broadcast.emit("updateRecInd", text)
      });
      socket.on("switchCam", (flatSwitchChecked) => {
        if (typeof broadcaster !== 'undefined') {
          socket.to(broadcaster).emit("switchCam", flatSwitchChecked);
        }
        console.log("switchCam");
      });
      socket.on("switchedCam", (videoSelect) => {
        console.log("index.js", videoSelect);
        socket.broadcast.emit("switchedCam", videoSelect);
      });
      socket.on("newVideo", (videoName) => {
        socket.broadcast.emit("newVideo", videoName);
      });
      socket.on("hideVideo", () => {
        socket.broadcast.emit("hideVideo");
      });
      socket.on("getVideo", () => {
        socket.broadcast.emit("getVideo");
      });
      socket.on("updateVideo", (videoName) => {
        socket.broadcast.emit("updateVideo", videoName);
      })
      
      socket.on("touchMove", (touchEvent) => {
        console.log(touchEvent);
      })

      socket.on("promptText", (promptText) => {
        socket.broadcast.emit("promptText",promptText);
      });
      socket.on("showWebm", (showWebm) => {
        socket.broadcast.emit("showWebm", showWebm);
      });
      socket.on("showPrompt", (showPrompt) => {
        socket.broadcast.emit("showPrompt", showPrompt);
      });

      socket.on("playWebm", () => {
        socket.broadcast.emit("playWebm");
      });
      
      socket.on("embodimentText", (embodimentText) => {
        socket.broadcast.emit("embodimentText",embodimentText);
      })
      
      socket.on("showSurvey", (showSurvey) => {
        socket.broadcast.emit("showSurvey", showSurvey);
      });
      socket.on("hideSurvey", () => {
        socket.broadcast.emit("hideSurvey");
      });
      socket.on("saveVidViewDict", (vidViewDict) => {
        console.log(vidViewDict);
        var curDate = new Date();  
        var dateStr = `${curDate.getFullYear()}${curDate.getMonth()+1}${curDate.getDate()}_`
            +`${curDate.getHours()}${curDate.getMinutes()}${curDate.getSeconds()}`
        fs.writeFile(`vidViewDict/${dateStr}.txt`, JSON.stringify(vidViewDict), function(err) {
          if (err) {
            console.log(err);
          }
        });
      });

    });

    server.listen(port);
}