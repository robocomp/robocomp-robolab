# Joystick Simulator Controller

The component was created to simulate a joystick to robot control. Component created in Python with using Qt.


## Configuration parameters
As any other component,
``` *joystickSimulatorController* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    DifferentialRobotProxy = differentialrobot:tcp -h localhost -p 10004
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd robocomp-robolab/components/joystickSimulatorController

``` <joystickSimulatorController 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:


```src/joystickSimulatorController.py etc/config ```

    --Ice.Config=config
