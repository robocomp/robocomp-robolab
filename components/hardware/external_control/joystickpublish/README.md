# JoystickPublishComp

`JoystickPublishComp` component is a C++ implementation for generic joystick used for controlling real or simulated robot. The component implements the interface `JoystickAdapter.idsl` to read raw joystick throttles and button events, then encapsulates into message type `TData` to serve across RoboComp.

The message type and parameter detail can be found in file `robocomp/interfaces/IDSLs/JoystickAdapter.idsl`.


## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). We can compile `JoystickPublishComp` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish/
cmake .
make
```

## Configuration parameters
`JoystickPublishComp` component parameters are characterized as the `etc/config` file described below:

```
CommonBehavior.Endpoints=tcp -p 10000


# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

InnerModelPath = innermodel.xml


JoystickAdapter=JoystickAdapter
joystickUniversal.NumAxes = 3
joystickUniversal.Axis_0 = advance, -500, 500, false  #left-right movement
joystickUniversal.Axis_1 = rotate, -500, 500, true  #front-left movement
joystickUniversal.Axis_2 = side, -500, 500, false
#joystickUniversal.Axis_2 = clavicula, -2, -1, 0


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

The config file is self-explained. We can define control range and behavior of each joystick axis via parameter `NumAxes` and `Axis_n`.  

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/JoystickPublish etc/config-run
```
## Known issues
