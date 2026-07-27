# BodyHandJointDetectorClient
This is a test client for [BodyHandJointDetector component](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/BodyHandJointsDetector).


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
cd ../../BodyHandJointsDetector
python src/BodyHandJointsDetector.py etc/config
```

2) Run the client:
```
cd ../test/bodyHandJointsDetectorClient
python src/BodyHandJointDetectorClient.py etc/config
```


## DEMO:

[![Youtube link](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/images/handbodyPoseYoutube.png)](https://www.youtube.com/watch?v=qpOU7or_zx8)

