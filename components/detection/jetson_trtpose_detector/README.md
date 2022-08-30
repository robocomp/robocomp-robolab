# jetson_detector
Intro to component here

Detects human skeleton using trtpose DNN from NVIDIA

## Configuration parameters
As any other component, *jetson_detector* needs a configuration file to start. In
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
cd <jetson_detector's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/jetson_detector config
```
