# Hardware Components

**Hardware components** contains various driver components for various robotics hardware, e.g robot base, camera, control equipment, Inertial Measurement Unit (IMU), laser sensors and motors. This category has these components:
- `base` folder contains:
  - `delfos` component is implemented as generic controller for omni-directional robot (e.g omni wheels robot).
  - `DifferentialRobotComp` component is implemented as generic controller for differential robot (e.g two wheels robot)
  - `RobexComp` component is a controller software for custom-made mobile robot in Robolab, University of Extremadura.
- `camera` folder contains:
  - `astraRGBD` component provides access to a Orbbec Astra device through the Astra SDK driver, serving depth and RGB image.
  - `CameraIP` component serves image across RoboComp ecosystem through TCP/IP protocol from a camera server.
  - `CameraSimple` component is implemented in Python as a basic driver for most type of camera hardware, to serve image data across RoboComp ecosystem.
- `external_control` folder contains:
  - `JoystickComp` component is a C++ implementation for generic joystick for controlling differential robots.
  - `JoystickOmni` component is a C++ implementation for generic joystick for controlling omni-directional robots.
  - `JoystickPublishComp` component is a C++ implementation for generic joystick used for controlling real or simulated robot.
  - `KeyboardRobotController` component is a Python implementation for controlling differential robots using keyboard.
- `imu` folder contains:
  - `phidgetimu` component is a C++ implementation of IMU driver.
  - `PyIMU` component is a Python version implementation of IMU driver.
- `laser` folder contains:
  - `hokuyo` component provides a wrapper for accessing data from [Hokuyo c_urg library](https://debian.pkgs.org/8/debian-main-amd64/liburg0-dev_0.8.18-2_amd64.deb.html) and serves the laser measurements.
- `motor` folder contains motor driver components for specific motors.

The detail of the description, installation, and configuration of each component can be found on the respective component folder.
