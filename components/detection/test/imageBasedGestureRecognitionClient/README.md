# ImageBasedGestureRecognitionClient
This is a test client for [ImageBasedGestureRecognition component](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/imageBasedGestureRecognition).


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
cd ../../imageBasedGestureRecognition
python src/ImageBasedGestureRecognition.py etc/config
```

2) Run the client:
```
cd ../test/imageBasedGestureRecognitionClient
python src/ImageBasedGestureRecognitionClient.py etc/config
```


## DEMO:

[![Youtube link](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/images/imagebasedRecognitionYoutube.png)](https://www.youtube.com/watch?v=aAywMZqqlVw)

