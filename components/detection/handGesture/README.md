# HandGesture Component

This component recognizes hand gesture from the image feed. It exposes **getHandGesture** method to the interface which takes image of hand, and it's hand pose (or *Hand Keypoints*) as input and outputs the recognized American Sign Language alphabet (non motion based only).

User can also set the classes from which recognition will take place using the *setClasses* method which takes ASL alpahbets subset as input and train the model accordingly.

*Hand Gesture Client* component can be used to conveniently visualize results of the component.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before using the component, the user needs to install the necessary libraries:
```
pip install numpy opencv-python pickle scikit-learn==0.22.2.post1
```
Note: Make sure your scikit-learn version is `0.22.2.post1`

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

After configuring proxies, 

Download pretrained model which detects hand gesture from set of all ASL alphabets from [here](https://drive.google.com/file/d/1ocBUyuf12k5COQ-2fOJEi-O8TQ_PcK-2/view?usp=sharing) and place it in the assets folder.

Note: This model is for faster processing when all classes are to be detected. Component will work fine even if not used.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd robocomp-robolab/components/detection/handGesture/
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
## About Dataset

Dataset used for training model maps hand keypoints to gesture. It is created using [ASL Alphabet](https://www.kaggle.com/grassknoted/asl-alphabet?) dataset which is available publically. The dataset contains 87,000 images which are 200x200 pixels. Images of this dataset were passed through Keypoint Detection module to obtain keypoint to gesture mappings.

In assets folder,
- `points.npy` contains hand keypoints
- `labels.npy` contains corresponding gestures

## Saving trained model

Trained model can be saved by setting `self.save_model=True`. Name and path of the saved file can also be changed by changing `self.model_name`. By default, model will not be saved.
