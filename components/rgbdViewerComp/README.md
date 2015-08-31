```
```
#
``` rgbdViewer
```
Intro to component here

Simple component to show how RGBD images from the RGBD.idsl interface can be displayed using Qt and OpenCV.

Please note that the config file has an extra line 

    Ice.MessageSizeMax=1000000

to tell Ice to use a bigger network buffer.

Also, a line has been included in src/CMakeListsSpecific.txt

    INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv2.cmake)

so CMake can take care of OpenCV dependencies.

## Configuration parameters
As any other component,
``` *rgbdViewer* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <rgbdViewer 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```rgbdViewer ```

    --Ice.Config=config
