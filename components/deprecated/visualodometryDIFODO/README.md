# Instructions:

For use this component, you can need install to MRPT and the openni2RGBD component locate in our Github repository robocomp-robolab/components/openni2RGBD/
```
mkdir ~/software
cd ~/software
git clone https://github.com/jlblancoc/mrpt
cd mrpt
mkdir build
cd build
ccmake ..
make
sudo make install
echo <path to MRPT> >> ~/.bashrc'
source ~/.bashrc

In openni2RGBD component:
mkdir build
cd build
cmake ..
make
bin/openni2RGBD --Ice.Config=../config


In this component:
mkdir build
cd build
cmake ..
make
bin/visualodometryDIFODO --Ice.Config=../config


If you receive this error, no problem. Only you must repeat the execution:
visualodometryDIFODO: ../../src/xcb_io.c:179: dequeue_pending_request: Assertion `!xcb_xlib_unknown_req_in_deq' failed.


Output example:
pose(x,y,z,yaw,pitch,roll)=(0.0009,-0.0013,1.5006,-0.11deg,0.07deg,-0.03deg)
pose(x,y,z,yaw,pitch,roll)=(0.0015,-0.0022,1.5005,-0.15deg,0.07deg,-0.02deg)
pose(x,y,z,yaw,pitch,roll)=(0.0016,-0.0090,1.5034,-0.06deg,0.27deg,0.26deg)
pose(x,y,z,yaw,pitch,roll)=(0.0020,-0.0093,1.5022,-0.01deg,0.38deg,0.15deg)
pose(x,y,z,yaw,pitch,roll)=(0.0024,-0.0091,1.5021,-0.07deg,0.38deg,0.12deg)
pose(x,y,z,yaw,pitch,roll)=(0.0023,-0.0094,1.5029,-0.09deg,0.37deg,0.14deg)
pose(x,y,z,yaw,pitch,roll)=(0.0032,-0.0095,1.5015,-0.22deg,0.67deg,-0.03deg)
pose(x,y,z,yaw,pitch,roll)=(0.0039,-0.0093,1.5004,-0.32deg,0.93deg,-0.21deg)
pose(x,y,z,yaw,pitch,roll)=(0.0041,-0.0082,1.5027,0.27deg,1.60deg,-0.10deg)
pose(x,y,z,yaw,pitch,roll)=(0.0024,-0.0050,1.5074,2.77deg,2.13deg,0.95deg)
pose(x,y,z,yaw,pitch,roll)=(-0.0001,0.0015,1.5120,5.55deg,2.62deg,2.73deg)
pose(x,y,z,yaw,pitch,roll)=(-0.0080,0.0009,1.5141,11.34deg,3.22deg,5.03deg)
pose(x,y,z,yaw,pitch,roll)=(-0.0158,0.0005,1.5132,15.56deg,2.22deg,6.29deg)
```



```
```
#
``` visualodometryDIFODO
```
Intro to component here


## Configuration parameters
As any other component,
``` *visualodometryDIFODO* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <visualodometryDIFODO 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```visualodometryDIFODO ```

    --Ice.Config=config
