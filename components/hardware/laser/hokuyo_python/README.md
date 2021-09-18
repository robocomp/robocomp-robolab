# hokuyo_python
Intro to component here

Component that provides access to the Hokuyo family of LIDARs

You need to clone this library:

https://github.com/pasuder/hokuyo-python-lib

Install it with suoo puthon3 setup.py install

## Configuration parameters
As any other component, *hokuyo_python* needs a configuration file to start. In
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
cd <hokuyo_python's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/hokuyo_python config
```
