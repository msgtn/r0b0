
## Appendix

### Telepresence implementation
WebRTC is a confusing crissing-crossing async shouting match not unlike a political debate on an uncle's Facebook status.
Here's a table of the signaling that I *think* is going on between the desktop page `blsm_broadcaster.js` and the mobile page `blsm_controller.js`.
The robot is on the broadcaster's end; the controller is the remote user moving their phone to remotely control the robot.
Time is going downwards.
| broadcaster | controller | what's going on |
|:------------|-----------:|:-|
| connect | connect | Both devices connect to the same socket |
| emits `broadcaster` | | The broadcaster signals that it has media to broadcast |
| | emits `watcher`| the watcher signals that can watch |
| handles `watcher` | | The broadcaster adds the controller as a new `RTCPeerConnection`, gets its own media devices (i.e. the camer and microphone connected to the robot's computer), and sends this information to the controller |
| emits `candidate` | | The broadcaster sends information of the shared ICE server that clients on the same socket will use |
| | handles `candidate` | The controller processes the ICE server information |
| creates and emits `offer` | | The broadcaster offers details of its streaming capabilities |
| | handles `offer` | The controller reads the broadcaster's stream and updates its video stream with the robot's camera feed | 
| | creates and emits `answer` | The controller shares details of its own streaming capabilities (i.e. for the remote controller's audio/video to come through on the broadcaster's robot) |
| handles `answer` | | The broadcaster updates its video stream with the controller's camera feed (probably the phone's front-facing camera) |


### Design goals
Blossom serves as a critical design that questions three facets of robotics.
The first is aesthetics.
Most robots are white and LED-illuminated; others including myself have spilled many LaTeX templates over the downsides of this aesthetic conformity.
The second is utilitarianism.
No, Blossom won't fold your clothes or clean your room or wash your dishes, but then again, no robot short of unobtainable research prototypes can.
The third is consumption of robots.
Consumer robots are advertised and sold as 
Apart from the inherent ills of advertisement which needs no further bludgeoning, the marketing of robots performing physical or mental feats way above their actual capabilities in overproduced promotional videos is actively hurting robot development.
