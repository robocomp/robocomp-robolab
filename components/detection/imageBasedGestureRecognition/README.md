# ImageBasedRecognition
In this project, we implement [WLASL recognizer](https://github.com/dxli94/WLASL). There are pretrained model for this dataset. Therefore, we reuse these models without any training. 
In the image-based approach, they use I3D model for recognition. Please follow instruction in this blog for updating the WLASL models [link](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/post05).

## Configuration parameters

The configuration is already set, if changing the port of this component, please change to the same port in this component's client 
[link](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/test/imageBasedGestureRecognitionClient)


The setting for python enviroment can be found [here](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/post04)


Copy pretrained model from this [link](https://drive.google.com/file/d/1VnCSZHCt69Cah7Xvf2D7P2Lfsx3L1Mqh/view?usp=sharing) to src/_model/ in the detector component folder.

## Starting the component


Need to CMAKE the code with this command:

```
cmake .
```

After editing the new config file we can run the component:

```
python src/ImageBasedGestureRecognition.py etc/config
```
