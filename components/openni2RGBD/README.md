#openNI2RGBDComp

This component provides accest to a Xtion or Kinect device through the OpenNI2 driver.


## Configuration parameters
As any other component,
``` *openNI2Comp* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <openNI2Comp 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```openNI2Comp ```

    --Ice.Config=config



YOU CAN DOWNLOAD A VERSION OF OPENNI 2.2 IN https://github.com/mhaut/openni2

file compile for Ubuntu x64 in https://github.com/mhaut/openni2/blob/master/Packaging/Final/OpenNI-Linux-x64-2.2.tar.bz2
