# HandDetection

`HandDetection` component can detect and track human hands through an image stream from an RGBD sensor using `RGBD.idsl`. The component implements the `HandDetection.idsl` interface, which provides two functions:
- **int addNewHand(int expectedHands, TRoi roi)**: adds new hand to track given region of interest (ROI) argument `TRoi roi`.
- **Hands getHands()**: returns current tracked hands. Type: `Hands`.
- **int getHandsCount()**: returns current number of tracked hands.

In general, the component has two functionalities: tracking new hands given ROI by calling `addNewHand()` function and reporting current tracked hands by calling `getHands()` function.

The component is implemented based on `Real-Time Hand Gesture Detection` library of Sasha Gazman, whose README file can be found [here](./src/libs/HandDetection/README.md). You can also find the details of data types and function templates of `HandDetection.idsl` interface in file `robocomp/interfaces/IDSLs/HandDetection.idsl`.


## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python
```

## Configuration parameters

As an example, `HandDetection` component parameters are characterized in config file described below:

```
# Endpoints for implemented interfaces
HandDetection.Endpoints=tcp -p 21222


# Proxies for required interfaces
RGBDProxy = rgbd:tcp -h localhost -p 10096
CameraSimpleProxy = camerasimple:tcp -h localhost -p 10005

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

Ice.MessageSizeMax=2000480
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
debug=false
flip=true
Ice.ACM.Server=10
```

In the config file, you can flip the image stream or not by switching the parameter `flip=true|false`. You also must ensure the proxies' hostname and port number of `RGBDProxy` and `CameraSimpleProxy` to request correctly RGB and depth image stream.
And, the port number of the parameter `HandDetection.Endpoints` must be the same as the corresponding number of the client component using the `HandDetection` component.


## Starting the component
To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cd ~/robocomp/components/robocomp-robolab/components/detection/handDetection/
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/handDetection.py etc/config-run
```
