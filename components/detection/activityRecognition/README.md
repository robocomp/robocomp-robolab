# ActivityRecognition

## Description

`ActivityRecognition` component predicts human activity based on the sequence of human skeleton gesture movements(set of human joints, which can also be referred to as the pose or state). The component implements the `ActivityRecognition.idsl` interface, which provides two functions:
- **addSkeleton()**: expects an array of shape `(3, 15)` for xyz-positions of
15 joints.
- **getCurrentActivity()**: returns a json-string of top activities with their probabilities.  

The component requires a minimum of 32 skeletons dataset to produce a prediction.  
The component can be run together with `activityRecognitionClient`, `poseEstimation` and `camerasimple` components to produce predictions based on the real-time input. The communication diagram is shown below:

![Communication diagram](HAR_interaction.png)

### Training activity recognition model

Inside of the component directory, you will find the training code using `PyTorch` for producing a convolutional neural network (```dl_training```) that is used by this component to produce predictions. If you would like to train your model, please follow the [training guide here](./dl_training/README.md). Once you have trained your model, replace the default model ```src/data/best.pth.tar``` with your ```*.pth.tar``` file.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy matplotlib PyTorch torchnet openpyxl sklearn
```

## Configuration parameters

As an example, `ActivityRecognition` component parameters are characterized in the config file described below:

```
# Endpoints for implemented interfaces
ActivityRecognition.Endpoints=tcp -p 12253



# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Note that we need to make sure the port number of the parameter `ActivityRecognition.Endpoints` is the same as the corresponding number of the client component using the `ActivityRecognition` component.


## Starting the component
To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cd ~/robocomp/components/robocomp-robolab/components/detection/activityRecognition/
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/activityRecognition.py etc/config-run
```

For developer notice, the component also contains `specificworkerSVM.py` which can be used instead of the default `specificworker.py`. It uses SVM instead of CNN. In this case, Scikit-Learn library is required. However, the SVM classifier does not generalize. There's also a ```SVM_hand_crafted``` folder inside of the component, which provides the code for feature extraction and fitting SVM model.
