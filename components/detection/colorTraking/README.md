
# ColorTracking

`ColorTracking` component is an interactive application that allows the user to track an area of color through an image stream from an RGBD sensor. The component uses `openCV` for color processing and interactive windows and `imutils` for transformations in image, it also implements `RGBD.idsl` interfaces to request image stream from an RGBD sensor (e.g Asus Xtion, Kinect, etc) for the tracking task.

For developer notice, the details of `RGBD.idsl` can be found in file `robocomp/interfaces/IDSLs/RGBD.idsl`.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install `openCV2`, `imutils` and `scipy` as follows:

```
pip install opencv-python imutils scipy
```

## Configuration parameters

As an example, `ColorTracking` component parameters are characterized in config file described below:

```
# Proxies for required interfaces
RGBDProxy = rgbd:tcp -h localhost -p 10096


# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

Ice.MessageSizeMax=2000480
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
debug=true
flip=true
```
The `RGBDProxy` parameter specifies the TCP/IP link that this component will use to received the streaming images. Also, the user can flip the image stream or not by switching the parameter `flip=true|false`.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cd ~/robocomp/components/robocomp-robolab/components/detection/colorTraking/
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/colorTracking.py etc/config-run
```

Then, you can click and drag on the GUI window to specify the area of color that you would like to track.
## Known issues
