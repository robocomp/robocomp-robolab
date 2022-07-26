
# CameraSimple

`CameraSimple` component is implemented in Python as a basic driver for most type of camera hardware, to serve image data across RoboComp ecosystem. This component uses `CameraSimple.idsl` interface, which provides method template `idempotent void getImage(out TImage im)` to return image data as type `TImage`. The `CameraSimple.idsl` interface specification can be found in file `robocomp/interfaces/IDSLs/CameraSimple.idsl`.


## Compiling and Installation
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `CameraSimple` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hardware/camera/camerasimple/
cmake .
make
```
## Configuration parameters
`CameraSimple` component parameters are characterized as the `etc/config` file described below:

```
# Endpoints for implemented interfaces
CameraSimple.Endpoints=tcp -p 10005


# This property is used by the clients to connect to IceStorm.
# TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

Note that we need to make sure the port number of the parameter `CameraSimple.Endpoints` is the same as the corresponding number of the client component using the `CameraSimple` component.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/camerasimple.py --Ice.Config=etc/config-run
```
## Known issues
