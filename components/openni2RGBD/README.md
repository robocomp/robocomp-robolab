#openNI2RGBDComp

This component provides accest to a Xtion or Kinect device through the OpenNI2 driver, offering the RGBD.idsl interface

You can download a versión of OpenNI2.2 for Ubuntu x64 from:
    
    https://github.com/mhaut/openni2
    
and execute with sudo de *install.sh* script to copy the relevant files to */usr*

or you can compile the whole library for Ubuntu x64 downloading de sources from:     

    https://github.com/mhaut/openni2/blob/master/Packaging/Final/OpenNI-Linux-x64-2.2.tar.bz2

Check that the include files and .so libraries are correctly copied to /usr or /usr/local

## Configuration parameters
As any other component,
``` *openNI2RGBDComp* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <openNI2RGBDComp 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/openNI2RGBDComp --Ice.Config=config



