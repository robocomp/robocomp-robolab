```
```
#
``` RealSensePoseEstimation
```
This component interfaces to the RealSense SDK 2.0
You need to dowload the debian package that contains the library "realsense2.so" in the system to compile and link the component
Get the software from: https://www.intelrealsense.com/developers/

## Configuration parameters
As any other component,
``` *RealSensePoseEstimation* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <RealSensePoseEstimation 's path> ```

    cp etc/config config

After editing the new config file we can run the component:

    bin/

```RealSensePoseEstimation ```

    --Ice.Config=config
