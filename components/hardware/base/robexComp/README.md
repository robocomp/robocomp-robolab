# RobexComp

`RobexComp` component is a controller software for custom-made mobile robot in Robolab, University of Extremadura. The controller component consists of `JoystickAdapter.idsl` for sending control data message, `DifferentialRobot.idsl` for reading and controlling specific dynamics of differential robot and `GenericBase.idsl` for reading general robot states. For developer notices, the following list specifies the set of implemented functions for each of the interfaces:

* **DifferentialRobot.idsl**: please refer to `DifferentialRobotComp` documentation in file `robocomp/components/robocomp-robolab/components/hardware/base/differentialrobotComp/`.
* **JoystickAdapter.idsl**:
  - **sendData(const TData  &data, const Ice::Current&)**: sends robot control data specified in `JoystickAdapter.idsl`. Type: `TData`.
* **GenericBase.idsl**:
  - **getBaseState(RoboCompGenericBase::TBaseState  &state, const Ice::Current&)**: reads current robot state via Ice protocols.
  - **getBasePose(int  &x,  int  &z,  float  &alpha, const Ice::Current&)**: reads current robot pose via Ice protocols.

More information about interface parameters or data message type can be found in respective files in interface folder `robocomp/interfaces/IDSLs/`.

## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `RobexComp` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hardware/base/delfos/
cmake .
make
```

## Configuration parameters
As any other component, *RobexComp* needs a configuration file to start. In `etc/config` you can find an example of a configuration file. We can find there the following lines:

```
#
# This property is used to configure the endpoints of the subscriber adapter.
#
DifferentialRobot.Endpoints=tcp -p 10004
GenericBase.Endpoints= tcp -p 10001
CommonBehavior.Endpoints=tcp -p 11004
JoystickAdapter.Endpoints=tcp -p 10104

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
#DRobot.Device=/dev/ttyUSB0
#DRobot.Handler = Robex

DRobot.Logger = local
DRobot.maxVelAdv = 200
DRobot.maxVelRot = 0.3

#Pulguita usa Maxon
#Speedy usa Faulhaber


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

The config file is self-explained, please note that we can switch between simulation and real operation by changing the parameter `DRobot.Device` and `DRobot.Handler`. Also note that we need to make sure the port number of the parameter `.Endpoints` is the same as the corresponding number of the client component using that endpoint.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
./bin/delfos etc/config-run
```
