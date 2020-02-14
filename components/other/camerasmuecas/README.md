
# CamerasMuecas

`CamerasMuecas` component captures two streams of image from two camera source and rotates them upside down (180 degree). This component serves the two image streams via `CamerasMuecas.idsl` interface, which provides method template `idempotent ImagePair getImages()` to return image data as type `ImagePair`. The `CamerasMuecas.idsl` interface specification can be found in file `robocomp/interfaces/IDSLs/CamerasMuecas.idsl`. The component is implemented in Python.


## Compiling and Installation
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `CamerasMuecas` component:
```
cd ~/robocomp/components/robocomp-robolab/components/other/camerasmuecas/
cmake .
make
```
## Configuration parameters
As an example, `CamerasMuecas` component parameters are characterized in config file described below:

```
# Endpoints for implemented interfaces
CamerasMuecas.Endpoints=tcp -p 10005


# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

Ice.MessageSizeMax = 2000000 # in bytes

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```
We increase the `Ice.MessageSizeMax` parameter for larger allowed message size, because the streaming data is image. Note that we need to make sure the port number of the parameter `CamerasMuecas.Endpoints` is the same as the corresponding number of the client component using the `CamerasMuecas` component.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/camerasmuecas.py etc/config-run
```
## Known issues
