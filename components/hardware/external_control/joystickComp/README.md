# JoystickComp

`JoystickComp` component is a C++ implementation for generic joystick for controlling differential robots. The component implements the interface `JoyStick.idsl` to read raw joystick throttles and convert them to translational and rotational velocities, in order to control differential robots via `DifferentialRobot.idsl` interface.

For developer notice, these functions will be implemented:
- **void writeJoyStickBufferedData(const JoyStickBufferedData &gbd, const Ice::Current&)**: sends raw throttles to internal component logic to be converted to control signals.
- **void readJoyStickBufferedData(JoyStickBufferedData &gbd, const Ice::Current&)**: reads raw throttles from joystick.

The `JoyStick.idsl` can be found in file `robocomp/interfaces/IDSLs/JoyStick.idsl`.
For detail of `DifferentialRobot.idsl` interface and its implemented component please find in `robocomp/components/robocomp-robolab/components/hardware/base/differentialrobotComp` document.

## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). We can compile `JoystickComp` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickComp/
cmake .
make
```

## Configuration parameters
`JoystickComp` component parameters are characterized as the `etc/config` file described below:

```
CommonBehavior.Endpoints=tcp -p 20002


# Endpoints for implemented interfaces
JoyStick.Endpoints=tcp -p 10002


# Proxies for required interfaces
DifferentialRobotProxy = differentialrobot:tcp -h localhost -p 12238

# User config parameters
# Example = value
Joy.Device = /dev/input/js0 # input port for joystick hardware
Joy.XMotionAxis = 0         # X axis code
Joy.YMotionAxis = 1         # Y axis code
Joy.SampleRate = 10         # probe rate of joystick
Joy.MaxSteering = 1.        # max converted rotational speed
Joy.MaxAdvance = 700.       # max converted translational speed

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0

```

Depending on application demands, we can regulate control signal converted from joystick throttles via parameters `MaxSteering, MaxAdvance`. For sending control signals to target robot, please input valid proxy url in parameter `DifferentialRobotProxy`.
Note that we need to make sure the port number of the parameter `JoyStick.Endpoints` is the same as the corresponding number of the client component using the `JoystickComp` component. User also has to determine which port that the joystick is connected to, and then change the parameter `Joy.Device` to that port.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/joystick etc/config-run
```
## Known issues
