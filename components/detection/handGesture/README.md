# HandGesture

This component recognizes hand gesture from the image feed. It exposes **getHandGesture** method to the interface which takes image of hand, and it's hand pose (or *Hand Keypoints*) as input and outputs the recognized American Sign Language alphabet (non motion based only).

*Hand Gesture Client* component can be used to conveniently visualize results of the component.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python
```

## Configuration parameters
As any other component, *HandGesture* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
HandGesture.Endpoints=tcp -p 10004

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd robocomp-robolab/components/detection/handKeypoint/
```
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
cmake .
make
python3 src/HandGesture.py etc/config-run
```
