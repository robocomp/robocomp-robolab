# JoystickPublishFalconComp

`JoystickPublishFalconComp` component is a C++ implementation for [Falcon Haptic Controller](https://hapticshouse.com/pages/novints-falcon-haptic-device) joystick used for controlling real or simulated robot. The component implements the interface `JoystickAdapter.idsl` to read raw joystick throttles and button events, then encapsulates into message type `TData` to serve across RoboComp.

The message type and parameter detail can be found in file `robocomp/interfaces/IDSLs/JoystickAdapter.idsl`.

## Compiling and Installation

### Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). Before compiling the component, we need to download and install `libnifalcon` library for **Falcon Haptic Controller**, which provides basic functionality to connect to the falcon and load firmware to the internal microcontroller.

```
cd ~
git clone https://github.com/qdot/libnifalcon.git
cd libnifalcon
mkdir build
cd build
cmake ..
make
sudo make install
```

### Compile and install

Finally, we can compile `JoystickPublishFalconComp` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublishfalconComp/
cmake .
make
```

## Configuration parameters
`JoystickPublishFalconComp` component parameters are characterized as the `etc/config` file described below:

```
CommonBehavior.Endpoints=tcp -p 11162

# Proxies for published topics
JoystickAdapterProxy = joystickadapter:tcp -h localhost -p 10162


# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

The config file is self-explained. Note that we need to make sure the port numbers of subscribers of the published topic are the same to the proxy `JoystickAdapterProxy`.  

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/JoystickPublishFalcon etc/config-run
```
## Known issues
