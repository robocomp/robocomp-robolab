# DepthEstimation

This component finds out **Depth Value** from the image feed. **getDepthEstimation** method takes image as input from camera and returns depth values of each pixel in form of numpy matrix with same size as of input image. These depth values help in locating any object w.r.t camera.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip3 install numpy opencv-python tensorflow
```

## Configuration parameters
As any other component, *DepthEstimation* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
DepthEstimation.Endpoints=tcp -p 10100

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/depthEstimation/
```

After editing the config file we can run the component:

```
cmake .
make
python3 src/DepthEstimation.py etc/config-run
```

