# SVD48VBase
Intro to component here

> [!CAUTION]
> If the motor type has been changed, use the manufacturer's software to update the number of pole pairs and calibrate the encoder.


## Configuration parameters
As any other component, *SVD48VBase* needs a configuration file to start. In
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
cd <SVD48VBase's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/SVD48VBase config
```
