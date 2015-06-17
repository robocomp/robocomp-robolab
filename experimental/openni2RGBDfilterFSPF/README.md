
Requisite:

Driver Openni2. You can download https://github.com/mhaut/openni2
file compile for Ubuntu x64 in https://github.com/mhaut/openni2/blob/master/Packaging/Final/OpenNI-Linux-x64-2.2.tar.bz2

Eigen:
sudo aptitude install libeigen3-dev


```
```
#
``` openni2RGBDfilterFSPF
```
Intro to component here


## Configuration parameters
As any other component,
``` *openni2RGBDfilterFSPF* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <openni2RGBDfilterFSPF 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```openni2RGBDfilterFSPF ```

    --Ice.Config=config
