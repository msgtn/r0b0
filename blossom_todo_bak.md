# TODO

## Cite / Ideas
- https://www.theverge.com/23539916/sony-ps5-accessibility-controller-leonardo-ces-2023
- https://www.ifixit.com/News/51614/framework-laptop-teardown-10-10-but-is-it-perfect
- Robots as bicycles, cars, things that communities can grow from, are utilitarian, but also usable for pure Play
  - the bicycle of the _______

## Clients
- Spin up a generic server and use the host name and ports as saved variables
- [ ] Access server
- [ ] send message to server and get a response
- [ ]  receive message from server and emit response
- [ ]  gracefully close

### MIDI

- [ ] pack midi message
- [ ] unpack midi message
- [ ] Send midi to server
- [ ] receive general Message from server

### Phone
- [ ] Access server
- [ ] send gyro data to server


20230127
event: device_motion
# data should be emittable by callign socketio.emit(event=event,**data)
:
data: 
  event: event
  x: 1
  y: 2
  z: 3
namespace: 
I think this means that the js emits should also pass the event explicitly,
i.e. mirror the function call of the python emit

- [ ] midi to keyboard shortcuts and mouse control