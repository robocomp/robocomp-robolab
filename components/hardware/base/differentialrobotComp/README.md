# DifferentialRobotComp

`DifferentialRobotComp` component is implemented as generic controller for differential robot (e.g two wheels robot) in RoboComp left-handed coordinate. For developer notices,  the following list specifies the set of implemented control functions as specified in interface `DifferentialRobot.ice`:

- **getBaseState(RoboCompGenericBase::TBaseState  &state)**: gets the robot state. Type: `TBaseState`.
- **getBasePose(int  &x,  int  &z,  float  &alpha)**: returns current robot pose (e.g x-z coordinate and orientation alpha).
- **setSpeedBase(const float  advx, const float advz, const float  rot)**: sets robot speed with translational velocity `(advx, advz)` and rotational velocity `rot`. For the case of differential robot, translational velocity is only effective in X direction.  
- **stopBase(const Ice::Current&)**: immediately stops robot. Robot is stopped applying 0 values in velocity parameters.
- **resetOdometer()**: resets odometry of robot to values x=0, z=0, alpha=0.
- **setOdometer(const RoboCompGenericBase::TBaseState  &state, const Ice::Current&)**: sets user-input odometry via `TBaseState` state type for the robot base. It also updates the internal model by calling function **correctOdometer**.
- **setOdometerPose(const int  x, const int  z, const float  alpha)**: sets user-input odometry in the [innermodel](https://github.com/robocomp/robocomp/blob/stable/doc/innermodel.md) to the values x,z,alpha.
- **correctOdometer(const int  x, const int  z, const float  alpha)**: sets user-input odometry in the [innermodel](https://github.com/robocomp/robocomp/blob/stable/doc/innermodel.md) to the values x,z,alpha.

More information about interface parameters can be found in file `robocomp/interfaces/DifferentialRobot.ice`.

## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).
If so, we can compile the `DifferentialRobotComp` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hardware/base/differentialrobotComp/
cmake .
make
```

## Configuration parameters
As any other component, *DifferentialRobotComp*
needs a configuration file to start. In `etc/config` you can find an example of a configuration file. We can find there the following lines:

```
#
# This property is used to configure the endpoints of the subscriber adapter.
#
DifferentialRobotComp.Endpoints=tcp -p 10004

# Remote servers proxies example

#RemoteProxy = remote:tcp -h remotehost -p 10001

# User config parameters
#Gazebo configuration
#DRobot.Device= robot::position_iface_0
#DRobot.Handler= Gazebo

#Player configuracion
#DRobot.Device= localhost:6665
#DRobot.Handler= Player

#Robex configuration
DRobot.Device=/dev/ttyUSB0 # specifies USB port that the motor driver is connected
DRobot.Handler = Robex     

DRobot.Logger = local
DRobot.maxVelAdv = 300     # max translational velocity allowed
DRobot.maxVelRot = 0.8     # max rotational velocity allowed

#Pulguita usa Maxon
#Speedy usa Faulhaber


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

#
# We want a faster ACM
#
Ice.ACM.Client=10
Ice.ACM.Server=10
```

The config file is self-explained, please note that we can switch between simulation and real operation by changing the parameter `DRobot.Device` and `DRobot.Handler`. Also note that we need to make sure the port number of the parameter `DifferentialRobotComp.Endpoints` is the same as the corresponding number of the client component using the `DifferentialRobotComp` component.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
./bin/differentialrobotComp etc/config-run
```
