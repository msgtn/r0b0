// let socketAddr = "https://r0b0.ngrok.io";
// let socketAddr = "https://192.168.1.5:8080";
// let socketAddr = "https://0.0.0.0:8080";
let socketAddr = `https://${window.location.hostname}`;
if (window.location.port) {
  socketAddr += `:${window.location.port}`;
}
