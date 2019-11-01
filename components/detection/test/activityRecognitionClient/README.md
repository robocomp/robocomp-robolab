# ActivityRecognitionClient

This component serves as a testing and monitoring component for `ActivityRecognition` component. It first requests the RGB frames from `CameraSimple`, which it passes to the `poseEstimation` component and receives back the 2D and 3D joints data of human state. It then communicates with `activityRecognition` component to request predictions about the human activities and also visualizes the pose.  
Overall, the whole pipeline involves 4 components: `activityRecognition`, `activityRecognitionClient`, `poseEstimation`, `cameraSimple`. see below the communication diagram to understand the process.

![Communication diagram](HAR_interaction.png)

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python
```


## Configuration parameters
As an example, `ActivityRecognitionClient` component parameters are characterized in config file described below:

```
# Proxies for required interfaces
ActivityRecognitionProxy = activityrecognition:tcp -h localhost -p 12253
PoseEstimationProxy = poseestimation:tcp -h localhost -p 12254
CameraSimpleProxy = camerasimple:tcp -h localhost -p 10005



# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

You must ensure the proxies' hostname and port number of `ActivityRecognitionProxy`, `PoseEstimationProxy` and `CameraSimpleProxy` for correct communication between components.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cd ~/robocomp/components/robocomp-robolab/components/detection/test/activityRecognitionClient/
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/activityRecognitionClient.py etc/config-run
```

## Starting monitoring Activity Recognition
If you would like to start all components at once, you can run the following script:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/test/activityRecognitionClient/
./start.sh
```
