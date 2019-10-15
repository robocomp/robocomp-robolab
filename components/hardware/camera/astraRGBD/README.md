# astraRGBD

This component provides access to a Orbbec Astra device through the Astra SDK driver, serving depth and RGB image through the `RGBD.idsl` interface. The `RGBD.idsl` interface specification can be found in file `robocomp/interfaces/IDSLs/RGBD.idsl`

You can download a version of the Astra SDK from here:
https://orbbec3d.com/develop/

Or you can pull the latest version from git:
https://github.com/orbbec/astra

After extracting, go to the install directory and execute it:

```shell
cd AstraSDK-*/install
sudo sh install.sh
```

Probably you want to compile and test the samples from the SDK. You can go to the examples directory, compile and test:
```shell
cd samples/
cmake .
make
./bin/SimpleStreamViewer-SFML
```

Depending on your current installation you may need to install some packages to get it compiled. To get the examples running you would probably need to install the `libsfml-dev` package.


## Configuration parameters
As any other component, *astraRGBD* needs a configuration file to start. In

    etc/config

you can find an example of a configuration file as follow:

    CommonBehavior.Endpoints=tcp -p 11111


    # Endpoints for implemented interfaces
    RGBD.Endpoints=tcp -p 10096
    HumanTracker.Endpoints=tcp -p 11666



    Ice.MessageSizeMax=20004800
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10

    depth=true
    color=true
    body=true
    point=true

At the bottom you can find the flags to read from the different streams that Astra camera supplies.

### Other related files
You can find the needed cmake module for astra on this path
`$ROBOCOMP/cmake/modules/astra.cmake`

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd $ROBOCOMP/components/robocomp-robolab/components/astraRGBD/
    cp etc/config etc/config-run

After editing the new config file we can run the component:

    ./bin/astraRGBD etc/config-run

## Final notes
`astraRGBD` is currently under development and hence only some of the main functions of the `RGBD.idsl` interface have been implemented.

## Known issues
In the directory `AstraSDK-v__XXX__-Linux/samples`, the command
```
./SimpleBodyViewer-SFML
```
sometimes fails silently showing just a black screen.
You can change the messages shown on on the terminal by the example modifying the `lib/astra.toml` and set
the parameter  **level = "trace"** or **level = "debug"**.  
If you do so, you can see all messages output from the processes done inside the `astraSDK` when launching the example.
Take a deep look to the messages shown. If you find a message like this:
```
lib/Plugins/libOrbbecBodyTracking.so System could not load library. Perhaps a dependency is missing.
```
it is possible that you have a problem with the `libpng` library. If you are using Ubuntu 18.04, the installed library is `libpng16`, but the required version is `libpng12`.
For more information, you can see the description of the problem on the Orbbec forum:  
https://3dclub.orbbec3d.com/t/resolved-liborbbecbodytracking-so-system-could-not-load-library-bodyviewer-sfml-example-with-astrasdk-2-0-8-ubuntu-17-10-starting-bodystream/1462

And you can download `libpng12` from here:
https://packages.ubuntu.com/xenial/amd64/libpng12-0/download
