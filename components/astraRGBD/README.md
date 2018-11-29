# astraRGBD

This component provides accest to a Orbbec Astra device through the Astra sdk driver, offering the RGBD.idsl interface

You can download a versión of the Astra SDK from here:
https://orbbec3d.com/develop/

Or you can download the last version from git:
https://github.com/orbbec/astra

After extracting, go to the install directory and execute it:

```shell
cd AstraSDK-*/install
sudo sh install.sh
```

Probably you want to compile and test the samples from the sdk. You can go to the examples directory, compile and test:
```shell
cd samples/
cmake .
make
./bin/SimpleStreamViewer-SFML
```

Depending on your current installation probably you would need to install some packages to get it compiled. To get teh examples running you would probably need to install the libsfml-dev package.


## Configuration parameters
As any other component, *astraRGBD* needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    CommonBehavior.Endpoints=tcp -p 11111

    # Endpoints for implemented interfaces
    RGBD.Endpoints=tcp -p 10096

    Ice.MessageSizeMax=20004800
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10

    depth=true
    color=true

##### Otros ficheros relacionados
You can find the needed cmake module for astra on this path
`$ROBOCOMP/cmake/modules/astra.cmake`

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd $ROBOCOMP/components/robocomp-robolab/components/astraRGBD/
    cp etc/config config

After editing the new config file we can run the component:

    ./bin/astraRGBD --Ice.Config=config

## Final notes
Just some of the main functions of the rgbd interface have been currently implemented.
It's under development.