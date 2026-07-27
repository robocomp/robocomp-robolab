# PoseBasedGestureRecognitionClient
This is a test client for [PoseBasedGestureRecognition component](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/PoseBasedGestureRecognition).


## Configuration parameters
Follow the instruction in the main component for installation.

## Starting the component

Need to CMAKE the code with this command:

```
cmake .
```

After editing the new config file we can run the component:

1) Run the main component:
```
cd ../../PoseBasedGestureRecognition
python src/PoseBasedGestureRecognition.py etc/config
```

2) Run the client:
```
cd ../test/poseBasedGestureRecognitionClient
python src/PoseBasedGestureRecognitionClient.py etc/config
```
