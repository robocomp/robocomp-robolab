```
```
#
``` RealSensePoseEstimation
```
This component interfaces to the RealSense SDK 2.0 and through it to a RealSense T265 SLAM camera.
From 20.04 on you need to download a tar source and compile it from
https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.247181511.299939129.1613056151-457352940.1613056151

Additional information can be found in:
Get the software from: https://www.intelrealsense.com/developers/

The component reads the 6D pose of the camera and transforms it to RoboComp coordinate system.
It publishes the data using the FullPoseEstimationPub.idsl topic and also takes RPC requests through the FullPoseEstimation.idsl interface.

## Configuration parameters
As any other component,
``` *RealSensePoseEstimation* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    serial: serial number of the T265 camera
    print: whether it prints to console the readings
    TODO: frequency


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <RealSensePoseEstimation 's path> ```

    cp etc/config config

After editing the new config file we can run the component:

    bin/

```RealSensePoseEstimation ```

    --Ice.Config=config
