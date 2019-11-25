
# CameraIPCpp

`CameraIPCpp` component serves image with `AprilTags` across RoboComp ecosystem through TCP/IP protocol from a camera server, which ensures connection reliability for data integrity. This component uses `CameraSimple.idsl` interface, which provides method template `idempotent void getImage(out TImage im)` to return image data as type `TImage`. The `CameraSimple.idsl` interface specification can be found in file `robocomp/interfaces/IDSLs/CameraSimple.idsl`. The component is implemented in C++, user should refer to `CameraIP` component for Python version.


## Compiling and Installation
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `CameraIPCpp` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hardware/camera/cameraipcpp/
cmake .
make
```
## Configuration parameters
As an example, `CameraIPCpp` component parameters are characterized in config file described below:

```
CommonBehavior.Endpoints=tcp -p 10011


# Endpoints for implemented interfaces
CameraSimple.Endpoints=tcp -p 12299

Camera = http://10.253.247.43:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=admin&pwd=opticalflow


InnerModelPath = innermodel.xml

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```
The `Camera` parameter specifies the TCP/IP link that this component will use to received the streaming images. Note that we need to make sure the port number of the parameter `CameraSimple.Endpoints` is the same as the corresponding number of the client component using the `CameraIPCpp` component.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/cameraipcpp --Ice.Config=etc/config-run
```
## Known issues
