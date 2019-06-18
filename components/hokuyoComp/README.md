
# hokuyo

Hokuyo laser sensor family is a low-cost laser sensor applied in many areas in robotics research such as navigation, etc. It provides range data in short distance from 0.4~5 meters for basic localization or SLAM task in small indoor environment.

The hokuyoComp component provides a wrapper for accessing data from [Hokuyo c_urg library](https://debian.pkgs.org/8/debian-main-amd64/liburg0-dev_0.8.18-2_amd64.deb.html) and publish the laser data over RoboComp environment using Ice middleware framework. The component uses `Laser.idsl` interface.

## Resolve dependencies
This section assumes user already installed RoboComp core library and pull Robolab's components according to this [README guide](https://github.com/robocomp/robocomp).

Before compiling the component, we must first resolve its dependencies which are the Hokuyo driver packages:

**For Ubuntu 14.04 Trusty Tahr**

```
sudo apt-get install -y liburg0-dev
```

**For Ubuntu 16.04 Xenial Xerus onward**

Because `liburg0-dev` package does not exist in official PPA from Ubuntu 16.04 onward, we should download deb package directly from Trusty repo and install it by `dpkg`.

```
sudo apt install -y libc6 libgcc1 libsdl-net1.2 libstdc++6 libsdl1.2debian
cd ~/Downloads
wget http://ftp.br.debian.org/debian/pool/main/u/urg/liburg0_0.8.18-2_amd64.deb
wget http://ftp.br.debian.org/debian/pool/main/u/urg/liburg0-dev_0.8.18-2_amd64.deb
sudo dpkg -i liburg0_0.8.18-2_amd64.deb liburg0-dev_0.8.18-2_amd64.deb
```

Then we can compile the `hokuyoComp` component:
```
cd ~/robocomp/components/robocomp-robolab/components/hokuyoComp/
cmake .
make
```

## Configuration parameters
Unlike any other component, `hokuyoComp` is under development and has not any config file yet. However, we can copy config file from `sickLaser` due to similarity of interfaces between laser sensors.

```
cp ../sickLaser/etc/config config
```

You can then add Hokuyo specific parameters according to `constants.h` file in the component folder. Example of the config file can be described as below:

```
CommonBehavior.Endpoints=tcp -p 10000


# Endpoints for implemented interfaces
Laser.Endpoints=tcp -p 10003

# Hokuyo sensor specific parameters
Laser.Driver=HokuyoURG
Laser.Device=/dev/ttyACM0
Laser.StartValue=0
Laser.EndValue=768
Laser.SkipValue=1
Laser.SampleRate=100
Laser.angleRes=0.00613593
Laser.angleIni=-2.356197
Laser.Cluster=1
Laser.minRange=40
Laser.maxRange=4094
Laser.TalkToBase=false
Laser.maxDegrees=240
Laser.staticConf=1


Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Note that the `Laser.Driver` and `Laser.Device` param can be different depended on your Hokuyo device name and current port that the sensor is connected to. For checking every USB ports that have been connected:
```
ls /dev/ttyACM*
```
Then find the corresponding port that Hokuyo sensor is connected to.

## Starting the component
After editing config file for matching with Hokuyo device name and desired parameters of your laser sensor, we plug the Hokuyo sensor in any USB port and start the component using these commands:

```
./bin/hokuyo --Ice.Config=config
```

## Known issues

The hokuyoComp is compiled error on Ubuntu 18.04 in file `generichandler.h` with the error:
```
In member function 'RoboCompGenericBase::TBaseState GenericLaserHandler getBaseState()':
error: cannot declare catch parameter to be abstract class type 'IceUtil::Exception'
  catch(IceUtil::Exception e)
```   
The simple workaround is bypassing the try-catch structure in function `RoboCompGenericBase::TBaseState getBaseState()`:

```c++
RoboCompGenericBase::TBaseState getBaseState()
{
  RoboCompGenericBase::TBaseState b;

  base->getBaseState(b);

  printf("seguimos...\n");
  return b;
}
```

Then the component is compiled successfully.
