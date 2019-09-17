```
```
#
``` DifferentialRobotComp
```

`DifferentialRobotComp` component is implemented as a generic controller for differential robot (e.g two wheels robot). The follow list specifies set of implemented control functions as specified in interface `DifferentialRobot.ice`:

- *getBaseState*: gets robot state in Gazebo environment.
- *getBasePose*: provides current robot pose (e.g x-y coordinate and orientation) in Gazebo environment.
- *setSpeedBase*: sets robot speed with translational and rotational velocity.
- *stopBase*: immediately stops robot.
- *resetOdometer*: resets odometry of robot to reference coordinate.
- *setOdometer*: sets user-input odometry for the robot base.
- *setOdometerPose*: sets user-input odometry via pose.

More information about input parameters of these function can be found in file `robocomp/interfaces/DifferentialRobot.ice`.

## Compiling and Installation

This section assumes user has already installed RoboComp core library and pull Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

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

The config file is self-explained, please note that we can switch between simulation and real operation by changing the parameter `DRobot.Device`. Also note that we need to make sure the port number of the parameter `DifferentialRobotComp.Endpoints` is the same as the corresponding number of the client component using the `DifferentialRobotComp` component.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
./bin/differentialrobotComp --Ice.Config=etc/config-run
```
