# JoystickOmni

`JoystickOmni` component is a C++ implementation for generic joystick for controlling omni-directional robots. The component implements the interface `JoyStick.idsl` to read raw joystick throttles and convert them to translational and rotational velocities, in order to control omni-directional robots via `OmniRobot.idsl` interface.

For developer notice, these functions will be implemented:
- **void writeJoyStickBufferedData(const JoyStickBufferedData &gbd, const Ice::Current&)**: sends raw throttles to internal component logic to be converted to control signals.
- **void readJoyStickBufferedData(JoyStickBufferedData &gbd, const Ice::Current&)**: reads raw throttles from joystick.

The `JoyStick.idsl` can be found in file `robocomp/interfaces/IDSLs/JoyStick.idsl`.
For detail of `OmniRobot.idsl` interface and its implemented component please find in `robocomp/components/robocomp-robolab/components/hardware/base/delfos` document.

## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).. We can compile `JoystickOmni` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickomni/
cmake .
make
```

## Configuration parameters
`JoystickOmni` component parameters are characterized as the `etc/config` file described below:

```
#
# This property is used to configure the endpoints of the subscriber adapter.
#
JoyStick.Endpoints=tcp -p 15002
CommonBehavior.Endpoints=tcp -p 25002

# Remote servers proxies example
OmniRobotProxy = omnirobot:tcp -h robonuc1.local -p 12238

# User config parameters
# Example = value
Joy.Device = /dev/input/js0 # input port for joystick hardware
Joy.XMotionAxis = 0
Joy.YMotionAxis = 1
Joy.ZMotionAxis = 2
Joy.SampleRate = 5
Joy.MaxSteering = 0.3
Joy.MaxAdvanceX = 100
Joy.MaxAdvanceZ = 100

# Component properties
#
#
# Warn about connection exceptions
#
Ice.Warn.Connections=0

#
#
# Network Tracing
#
# 0 = no network tracing
# 1 = trace connection establishment and closure
# 2 = like 1, but more detailed
# 3 = like 2, but also trace data transfer
#
Ice.Trace.Network=0

#
# Protocol Tracing
#
# 0 = no protocol tracing
# 1 = trace protocol messages
#
Ice.Trace.Protocol=0
```

Depending on application demands, we can regulate control signal converted from joystick throttles via parameters `MaxSteering, MaxAdvanceX, MaxAdvanceZ`. For sending control signals to target robot, please input valid proxy url in parameter `OmniRobotProxy`.
Note that we need to make sure the port number of the parameter `JoyStick.Endpoints` is the same as the corresponding number of the client component using the `JoystickOmni` component. User also has to determine which port that the joystick is connected to, and then change the parameter `Joy.Device` to that port.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/joystickomni etc/config-run
```
## Known issues
