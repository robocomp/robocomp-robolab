# RicohOmni

Installation of Ricoh drivers

Goto: https://codetricity.github.io/theta-linux/software/  and choose the

    Using gstreamer and OpenCV without v4l2loopback

option, and install https://github.com/nickel110/gstthetauvc

Install de NVidia GStreamer plugin with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200

Add to .bashrc and source:
    export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc


## Configuration parameters
As any other component, *RicohOmni* needs a configuration file to start. In
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
cd <RicohOmni's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/RicohOmni config
```
