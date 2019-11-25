
# CamerasMuecasClient

`CamerasMuecasClient` component receives two images as type `ImagePair` from `CamerasMuecas` component and shows these images on a openCV GUI. Specifically, this component uses `CamerasMuecas.idsl` interface to receive `ImagePair`. The `CamerasMuecas.idsl` interface specification can be found in file `robocomp/interfaces/IDSLs/CamerasMuecas.idsl`. The component is implemented in Python.


## Compiling and Installation
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `CamerasMuecasClient` component:
```
cd ~/robocomp/components/robocomp-robolab/components/other/camerasmuecasclient/
cmake .
make
```
## Configuration parameters
As an example, `CamerasMuecasClient` component parameters are characterized in config file described below:

```
# Endpoints for implemented interfaces
# Proxies for required interfaces
CamerasMuecasProxy = camerasmuecas:tcp -h localhost -p 10005



# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.MessageSizeMax = 2000000

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```
We increase the `Ice.MessageSizeMax` parameter for larger allowed message size, because the receiving data is image. Note that we need to make sure the parameter `CamerasMuecasProxy` to have hostname and port correctly from `CamerasMuecas` endpoint.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/camerasmuecasclient.py etc/config-run
```
## Known issues
