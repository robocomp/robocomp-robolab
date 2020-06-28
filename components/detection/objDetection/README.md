# objDetection

This component allows the user to detect objects in the camera feed in real time. It uses [Darknet YOLO v4](https://github.com/alexeyab/darknet#requirements) in the backend to identify the objects and predict the bounding box. Currently the model is trained on MS COCO dataset and can identify 80 object categories. For object categories please refer to this [link](https://github.com/AlexeyAB/darknet/blob/master/data/coco.names).

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

For dependencies, please ensure that the dependencies are satisfied as mentioned on the github of [Darknet YOLO v4](https://github.com/alexeyab/darknet#requirements).

Next download the Darknet Yolo v4 from the [following link](https://drive.google.com/file/d/1I0RU4PEMOUQ-rDlXw9cndGx958HztZUj/view?usp=sharing). Use this model only to avoid debugging and resolving path issues. Place the zip file in the folder `detection/objDetection` and extract there.

After dependencies are installed build the Darknet Yolo model using following commands:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/objDetection/assets
make
```

Refer to the Makefile for the build specifications if required.

## Configuration parameters
As an example, `objDetection` component parameters are characterized in config file described below:

```
# Proxies for required interfaces
CameraSimpleProxy = camerasimple:tcp -h localhost -p 10005

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

You must ensure the proxies, hostname and port number of `CameraSimpleProxy` match the endpoints in the config files of the corresponding interfaces.
    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/objDetection
```
```
cp etc/config etc/config-run
```

After editing the new config file, build the component by:

```
cmake .
make
```

Now in order to run the component, follow these below steps. Ensure that the model is build and weights downloaded and placed in the correct folder as mentioned above.

Open one more terminal.

Terminal 1:
```
cd hardware/camera/camerasimple
python src/camerasimple.py etc/config-run
```

Terminal 2:
```
cd detection/objDetection
python src/objDetection.py etc/config-run
```

Once the component is running, user can see the camera feed with bounding box across the objects in a pop-up window.