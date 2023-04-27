# ricoh_z1
Intro to component here


## Configuration parameters
As any other component, *ricoh_z1* needs a configuration file to start. In
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
cd <ricoh_z1's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/ricoh_z1 config
```

https://codetricity.github.io/theta-linux/
instalar libuvc-theta pero no v4l2loopback usaremos gstthetauvc
hacer caso a home, ultima parte de equipament (gstreamer plug-in) para decodificaron por nvidia y ultimas dos partes de software
para decodificación por nvidia importante version de repo y este enlace https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200

por terminal (~250ms)
gst-launch-1.0 thetauvcsrc mode=2K   ! queue   ! h264parse   ! nvdec   ! queue   ! glimagesink sync=false 
por opencv (~500ms)
thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=0 ! video/x-raw,format=BGR ! queue ! appsink