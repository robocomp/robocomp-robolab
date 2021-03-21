# Depth Estimation Client Component

These component have various functions described below:

- It acts as testing and monitoring module for the complete `Depth Estimation Prediction Pipeline` which involves for components i.e `Camerasimple`, `DepthEstimationClient` and `DepthEstimation` 
- It acts as a Blackbox to interact with users or other components to obtain depth value or Predicted Depth


## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python tensorflow
```

## Configuration parameters
As any other component, *DepthEstimationClient* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
DepthEstimationClient.Endpoints=tcp -p 10200


# Proxies for required interfaces
CameraSimpleProxy = camerasimple:tcp -h localhost -p 10005
DepthEstimationProxy = depthestimation:tcp -h localhost -p 10100


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.Trace.Network=2
```
You must ensure the proxies' hostname and port number of `CameraSimpleProxy` and `DepthEstimationProxy` match the endpoints in the config files of the corresponding interfaces

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/test/depthEstimationClient/

```

After editing the new config file we can run the component:

```
cmake .
make
python3 src/DepthEstimationClient.py etc/config-run
```
Make sure that all other required components (`CameraSimple` and `DepthEstimation`) are up and running.
