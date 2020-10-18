# Hand Gesture Client Component

These component have various functions described below:

- It acts as testing and monitoring module for the complete `Hand Gesture Recognition Pipeline` which involves for components i.e `Camerasimple`, `HandGestureClient`, `HandGesture` and `HandKeypoint`
- It acts as a Blackbox to interact with users or other components to obtain Hand Bounding Box, Keypoints or Recognized Gesture


## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python
```

## Configuration parameters
As any other component, *HandGestureClient* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
HandGestureClient.Endpoints=tcp -p 10006


# Proxies for required interfaces
CameraSimpleProxy = camerasimple:tcp -h localhost -p 10005


# This property is used by the clients to connect to IceStorm.
# TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```
You must ensure the proxies' hostname and port number of `CameraSimpleProxy` match the endpoints in the config files of the corresponding interfaces

After configuring proxies, 

For detection using mediapipe, download hand detection models from [here](https://drive.google.com/file/d/1kfcuH4ZiWsCY14wXPyZQH1gNPSLa15em/view?usp=sharing), move it to the assets folder and unzip to get the models directory. Also, set **self.method = 'Mediapipe'** in `src/specificworker.py`

Optional:

For detection using SSD and MobileNet architecture download hand detection models from [here](https://drive.google.com/file/d/1DNytkeURTOvz6HQPlhEctNQt8T92uxAL/view?usp=sharing), move it to the assets folder and unzip to get the models directory. Also, set **self.method = 'SSD'** in `src/specificworker.py`

Note: This method is optional, by default mediapipe method should be used. Also, Hand Gesture Recognition may not work well with this method

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd ~/robocomp/components/robocomp-robolab/components/detection/test/handGestureClient/
```
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
cmake .
make
python3 src/HandGestureClient.py etc/config-run
```
Make sure that all other required components (`CameraSimple` and `HandGesture`) are up and running.

After running the component, you will be asked to enter set of ASL alphabets from which you want to get the gestures recognized.