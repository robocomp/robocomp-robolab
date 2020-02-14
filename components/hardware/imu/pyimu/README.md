
# PyIMU

`PyIMU` component is a Python version implementation of Inertial Measurement Unit (IMU) driver. The component uses `IMU.idsl` interface to serve combined message type `DataImu` consisted of component data type `Acceleration,  Gyroscope, Magnetic, Orientation, temperature`, which are measured from IMU sensor. In addition, `PyIMU` also utilizes `IMUPub.idsl` interface as a proxy to publish message type `DataImu` across RoboComp.  

For developer notice, these functions will be implemented:
- **getDataImu()**: returns combined message. Type `DataImu`.
- **getAcceleration()**: returns linear accelerations of each axis of the IMU 3D cartesian coordinate. Type: `Acceleration`.
- **getAngularVel()**: returns angular velocities of each axis of the IMU 3D cartesian coordinate. Type: `Gyroscope`.
- **getMagneticFields()**: returns magnetic field strength components of each axis of the IMU 3D cartesian coordinate. Type: `Magnetic`.
- **getOrientation()**: returns current angle values of yaw, pitch, row axis of the IMU coordinate. Type: `Orientation`.
- **resetImu()**: resets all measured values of the IMU to zeros.

The detail data type components can be found in file `robocomp/interfaces/IDSLs/IMU.idsl`.


## Compiling and Installation

### Resolving dependencies
This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). Before compiling the component, user needs to install *PySide2* as follows:

```
pip install PySide2
```

### Compile the component

Finally, we can compile `PyIMU` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/imu/pyimu/
cmake .
make
```

## Configuration parameters
`PyIMU` component parameters are characterized as the `etc/config` file described below:

```
IMU.Endpoints=tcp -p 10020

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999


device=/dev/ttyACM0

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

Note that we need to make sure the port number of the parameter `IMU.Endpoints` is the same as the corresponding number of the client component using the `PyIMU` component. User also has to determine which port that the IMU sensor is connected to, and then change the parameter `device` to that port.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/pyimu.py etc/config-run
```
## Known issues
