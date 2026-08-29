# BodyHandJointsDetector
Body and hand detection: I apply the lightweight Openpose model, MediaPipe library, and the optical flow algorithm. The combination can process in real-time and with a lightweight model, so it is suitable for edge devices.


## Configuration parameters
As any other component, *BodyHandJointsDetector* needs a configuration file to start. In
```
etc/config
```

The configuration is already set, if changing the port of this component, please change to the same port in this component's client 
[link](https://github.com/robocomp/robocomp-robolab/tree/master/components/detection/test/bodyHandJointsDetectorClient)


The setting for python enviroment can be found [here](https://robocomp.github.io/web/gsoc/2021/posts/trung_ngo_tan/post04)


Copy pretrained model from this [link](https://drive.google.com/file/d/1W3Ud3u_55pJ0V4ODs47pne_OUKTDLkQa/view?usp=sharing) to src/_model/ in the detector component folder.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <BodyHandJointsDetector's path> 
```
```
cp etc/config config
```

Need to CMAKE the code with this command:

```
cmake .
```

After editing the new config file we can run the component:

```
python src/BodyHandJointsDetector.py etc/config
```

