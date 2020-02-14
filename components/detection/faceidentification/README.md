# FaceIdentification

`FaceIdentification` component recognizes human faces and tags them according to training dataset. The component implements `FaceIdentification.idsl` interface, which  provides necessary functions and datatypes to communicate with other components in RoboComp ecosystem. The component uses `tensorflow` for **training** and **inference** phases.

For **training** models, you must first download the model files according to this [guide](Model.txt) and save them inside the folder `assets/`. You can find the training and model evaluation code in that folder.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python==4.1.1.26 scipy sklearn
```
To install `tensorflow`, most of the cases you only need to type in:
```
pip install tensorflow
```
However, if some errors happen, please consult on the [TensorFlow installation guide](https://www.tensorflow.org/install/pip).

## Configuration parameters

As an example, `FaceIdentification` component parameters are characterized in config file described below:

```
# Endpoints for implemented interfaces
FaceIdentification.Endpoints=tcp -p 10008



# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Note that we need to make sure the port number of the parameter `FaceIdentification.Endpoints` is the same as the corresponding number of the client component using the `FaceIdentification` component.

## Starting the component
To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cd ~/robocomp/components/robocomp-robolab/components/detection/activityRecognition/
cp etc/config etc/config-run
```

After editing the new config file we can run the component as follow:

```
cd robocomp/components/robocomp-robolab/components
```
Open 2 new terminals.

Terminal 1:
```
cd hardware/camera/camerasimple
python src/camerasimple.py etc/config-run
```

Terminal 2:
```
cd detection/faceidentification
python src/faceidentification.py etc/config-run
```

Terminal 3:
```
cd detection/test/faceidentificationclient
python src/faceidentificationclient.py etc/config-run
```

Now one can see the captured frames with a bounding box and person's name on top of the bounding box. Another popup window will open which allows the user to Add/Delete labels. One can also add an image directly from the camera feed and recognize faces in the images. Images are stored in the folder `robocomp/components/robocomp-robolab/components/detection/test/faceidentificationclient`.
