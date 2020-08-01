# HandKeypoint

This component finds out coordinates of **Hand Keypoints** from the image feed. **getKeypoints** method takes image and bounding box of detected hand as input and returns a sequence of sequence consisting of 21 2D coordinates of the hand keypoints. These 21 keypoints consist of four keypoints for each finger and one for the wrist.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python
```

User also need to install **openpose** to run this component. For installing openpose follow instructions mentioned in [Openpose Installation guide](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md). Please make sure `BUILD_PYTHON` flag is set to **true** during `cmake`

## Configuration parameters
As any other component, *HandKeypoint* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
HandKeypoint.Endpoints=tcp -p 10007

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/handKeypoint/
```
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
cmake .
make
python3 src/HandKeypoint.py etc/config-run
```
