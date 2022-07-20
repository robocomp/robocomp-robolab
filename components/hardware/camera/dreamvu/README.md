# Dreamvu
This component interfaces the DreamVu omni camera

Instructions to make it compile in 18.04 without RoboComp installed

- Comment in src/CMakeList.txt  
    #ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple CameraRGBDSimplePub)
    #ROBOCOMP_ICE_TO_SRC( CommonBehavior CameraRGBDSimple CameraRGBDSimplePub)
- Copy from 20.04 al needed .ice and .h and .cpp for the interfaces used in the
- add to CMakeListSpecific.txt the ICe libraries: 
    Ice++11 IceStorm++11
    and source files  
       CameraRGBDSimple.cpp
       CameraRGBDSimplePub.cpp
       CommonBehavior.cpp


## Configuration parameters
As any other component, *Dreamvu* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <Dreamvu's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/Dreamvu config
```
