# delfos

`delfos` component is implemented as generic controller for omni-directional robot (e.g omni wheels robot) in RoboComp left-handed coordinate. The left-handed coordinate has the positive x, y and z axes point right, up and forward, respectively (see picture below. Credit: https://www.oreilly.com). Positive rotation is clockwise about the axis of rotation.

![coordinate](https://www.oreilly.com/library/view/learn-arcore-/9781788830409/assets/a465e4c5-b6ca-4006-a40e-1aa9ad2ebc5d.png)

For developer notices, the following list specifies the set of implemented functions for controlling and robot state measuring as specified in interface `OmniRobot.idsl`:

- **getBaseState(RoboCompGenericBase::TBaseState  &state)**: gets the robot state. Type: `TBaseState`.
- **getBasePose(int  &x,  int  &z,  float  &alpha)**: returns current robot pose (e.g x-z coordinate and orientation alpha).
- **setSpeedBase(const float  advx, const float advz, const float  rot)**: sets robot speed with translational velocity `(advx, advz)` and rotational velocity `rot`. For the case of omnidirectional robot, translational velocity is effective with all directions in X-Z coordinate.  
- **stopBase(const Ice::Current&)**: immediately stops robot. Robot is stopped applying 0 values in velocity parameters.
- **resetOdometer()**: resets odometry of robot to values x=0, z=0, alpha=0.
- **setOdometer(const RoboCompGenericBase::TBaseState  &state, const Ice::Current&)**: sets user-input odometry via `TBaseState` state type for the robot base. It also updates the internal model by calling function **correctOdometer**.
- **setOdometerPose(const int  x, const int  z, const float  alpha)**: sets user-input odometry in the [innermodel](https://github.com/robocomp/robocomp/blob/stable/doc/innermodel.md) to the values x,z,alpha.
- **correctOdometer(const int  x, const int  z, const float  alpha)**: sets user-input odometry in the [innermodel](https://github.com/robocomp/robocomp/blob/stable/doc/innermodel.md) to the values x,z,alpha.

More information about interface parameters can be found in file `robocomp/interfaces/IDSLs/OmniRobot.ice`.

## Compiling and Installation

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

If so, we can compile the `delfos` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hardware/base/delfos/
cmake .
make
```

## Configuration parameters
As any other component, *delfos*
needs a configuration file to start. In `etc/config` you can find an example of a configuration file. We can find there the following lines:

```
CommonBehavior.Endpoints=tcp -p 12321


# Endpoints for implemented interfaces
OmniRobot.Endpoints=tcp -p 12238



Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Note that we need to make sure the port number of the parameter `OmniRobot.Endpoints` is the same as the corresponding number of the client component using the `delfos` component.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:

```
./bin/delfos etc/config-run
```
