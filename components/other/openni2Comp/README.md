
# openni2Comp

`openni2Comp` is a RoboComp component driver for depth camera families. It derives from `OpenNI2` library, which is a framework for Natural Interaction (NI) devices such as Kinect, Asus Xtion, etc. NI Devices or Natural Interfaces are devices that capture body movements and sounds to allow for a more natural interaction of users with computers. The component also implements Human Tracker mechanisms utilizing motion tracking middleware called **NITE**. In addition, this component also incorporates Intel Integrated Performance Primitives (Intel IPP), which is a software library that provides a broad range of functionality, including general signal and image processing, computer vision and data compression.

Regarding to RoboComp ecosystem, the component implements `RGBD.idsl` interface for serving RGB image as type `ColorSeq`, depth image as type `DepthSeq` and 3D coordinate as type `PointSeq`. For Human Tracker mechanisms, the component implements `HumanTracker.idsl` interface to detect and recognize multiple human identities as type `PersonList`, as well as their pose state.  

For developer notice, many template functions and data types for the `RGBD.idsl` and `HumanTracker.idsl` interface can be found respectively in folder `robocomp/interfaces/IDSLs/`.

## Resolve dependencies
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). This installation section is tested with Ubuntu 18.04.

Before compiling the component, we must first install `OpenNI2` and `NiTE2` library dependencies by following this [guide](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openni-nite.md).

After that, we install the newest `Intel IPP` library by following these steps:
1. Add Intel public key PPA
```
sudo bash
# <type your user password when prompted.  this will put you in a root shell>
# cd to /tmp where this shell has write permission
cd /tmp
# now get the key:
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
# now install that key
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
# now remove the public key file exit the root shell
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
exit
```

2. Add the Intel IPP repositories:

```
sudo sh -c 'echo deb https://apt.repos.intel.com/ipp all main > /etc/apt/sources.list.d/intel-ipp.list'
sudo apt update
```

3. Install the newest Intel IPP:

```
sudo apt install intel-ipp-64bit-2019.5-075
```

After installing all the libraries, we must export for CMake to find these libraries for compiling:

```
export NITE2=~/src/NiTE-Linux-x64-2.2
export OPENNI2_INCLUDE=~/repos/OpenNI2/Include
export OPENNI2_LINK=~/repos/OpenNI2/Bin/x64-Release/
export IPPROOT=/opt/intel/ipp/
```

Finally, we can compile the `openni2Comp` component.

```
cd ~/robocomp/components/robocomp-robolab/components/other/openni2Comp/
cmake .
make
```


## Configuration parameters
As an example, `openni2Comp` component parameters are characterized in config file described below:

```
#
# This property is used to configure the endpoints of the subscriber adapter.
#
RGBDComp.Endpoints=tcp -p 10096
HumanTrackerComp.Endpoints=tcp -p 10183
CommonBehavior.Endpoints=tcp -p 12213

# Remote servers proxies example
#RemoteProxy = remote:tcp -h remotehost -p 10001

InnerModelManagerProxy = innermodelmanager:tcp -h localhost -p 11175
```

Note that we must ensure the component clients, which are connected to the `RGBDComp` or `HumanTrackerComp`, have the correct port endpoints.

## Starting the component
We assume that the user has configured the necessary hardware such as Kinect, etc. 
To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/generic_config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/openni2 etc/config-run
```

## Known issues
