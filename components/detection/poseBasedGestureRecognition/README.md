# PoseBasedRecognition
For pose-based reocngition, we reuse Pose-TGCN (graph neural network). This model have body/hand joints input and output the gesture classes.

## Configuration parameters


The configuration is already set, if changing the port of this component, please change to the same port in this component's client 
[link](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/test/poseBasedGestureRecognitionClient)


The setting for python enviroment can be found [here](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/post04)


Copy pretrained model from this [link](https://drive.google.com/file/d/1noNAokd8a39a1_DbOjLn5Hdi9LsI4snr/view) to src/_model/ in the detector component folder.

## Starting the component


Need to CMAKE the code with this command:

```
cmake .
```

After editing the new config file we can run the component:

```
python src/PoseBasedGestureRecognition.py etc/config
```

