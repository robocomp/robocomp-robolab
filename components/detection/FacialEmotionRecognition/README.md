# FacialEmotionRecognition

Note: This component is under development. Currently it can generate a bounding box across the facial images and display emotion as "UNKNOWN". Once the model is completed, it will display the facial emotion.

## Resolving dependencies

face_recognition

## Configuration parameters
As an example, `FacialEmotionRecognition` component parameters are characterized in config file described below:

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

Open one more terminal.

Terminal 1:
```
cd hardware/camera/camerasimple
python src/camerasimple.py etc/config-run
```

Terminal 2:
```
cd detection/FacialEmotionRecognition
python src/FacialEmotionRecognition.py etc/config-run
```

Once the component is running, user can see the camera feed with bounding box across the faces in a pop-up window.