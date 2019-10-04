
# phidgetimu

`phidgetimu` component is a C++ version implementation of Inertial Measurement Unit (IMU) driver. The component uses `IMU.idsl` interface to serve combined message type `DataImu` consisted of component data type `Acceleration,  Gyroscope, Magnetic, Orientation, temperature`, which are measured from IMU sensor.

For developer notice, these functions will be implemented:
- **getDataImu()**: returns combined message type `DataImu`.
- **getAcceleration()**: returns linear accelerations type `Acceleration` of each axis of the IMU 3D cartesian coordinate.
- **getAngularVel()**: returns angular velocities type `Gyroscope` of each axis of the IMU 3D cartesian coordinate.
- **getMagneticFields()**: returns magnetic field strength components type `Magnetic` of each axis of the IMU 3D cartesian coordinate.
- **getOrientation()**: returns current angle values type `Orientation` of yaw, pitch, row axis of the IMU coordinate.
- **resetImu()**: resets all measured values of the IMU to zeros.

The detail data type components can be found in file `robocomp/interfaces/IDSLs/IMU.idsl`.


## Compiling and Installation

### Resolving dependencies
This section assumes user has already installed RoboComp core library and pull Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). Before compiling the component, user needs to install *phidget21* IMU driver. Please download the dependency package, which is valid for Ubuntu 14.04 onward:

```
sudo apt update
sudo apt install libusb-1.0-0-dev
```
Note that `libusb-1.0` may be already on your system, but the development libraries probably aren't. Then we can download *phidget21*:

```
cd ~
wget https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget.tar.gz
tar -xvzf libphidget.tar.gz
```

and install:

```
cd libphidget-2.1.9.20190409
./configure
make
sudo make install
```

### Compile the component

Finally, we can compile `phidgetimu` to start using the component:

```
cd ~/robocomp/components/robocomp-robolab/components/hardware/imu/phidgetimu/
cmake .
make
```

## Configuration parameters
`phidgetimu` component parameters are characterized as the `etc/config` file described below:

```
CommonBehavior.Endpoints=tcp -p 1653

# Endpoints for implemented interfaces
IMU.Endpoints=tcp -p 10066

device=/dev/ttyACM0

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Note that we need to make sure the port number of the parameter `IMU.Endpoints` is the same as the corresponding number of the client component using the `phidgetimu` component. User also has to determine which port that the IMU sensor is connected to, and then change the parameter `device` to that port.

## Starting the component

To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
./bin/phidgetimu etc/config-run
```
## Known issues
