# lidar_3d
Intro to component here

## Installation
    Install repo [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)

## Run
Set ip of the running computer in config and in lidar, by accessing the lidar ip in a browser, section Setting.
    
The difference between lidars is the ports:
    Helios
    - MSOP = 6699
    - DIFOP = 7788
    Pearl
    - MSOP = 6698
    - DIFOP = 7787


## Configuration parameters
As any other component, *lidar_3d* needs a configuration file to start. In
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
cd <lidar_3d's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/lidar_3d config
```
