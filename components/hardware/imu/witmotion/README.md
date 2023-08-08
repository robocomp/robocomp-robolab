# witmotion

This component uses the **WitMotion SDK**:
- https://github.com/WITMOTION/WitStandardProtocol_JY901
- https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/sdk
With the registers of https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol

You need to install pyserial with ```pip install pyserial``` and grant permissions to the port with udev rules or commands like ```sudo chmod 666 /dev/ttyUSB0```.

## Configuration parameters
As any other component, *witmotion* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <witmotion's path> 
```

After editing the new config file we can run the component:

```
src/witmotion.py etc/config
```
