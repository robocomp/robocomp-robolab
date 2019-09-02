# speechComp
This component is used as a Text to Speech component. It depends on the festival package that can be installed on Ubuntu with :  
```bash
sudo apt install festival
```

The default languaje for festival is configured for English speech. If you want an Spanish speech you can download the Junta de Andalucía generated that can be found in this repository on github:
https://github.com/guadalinex-archive/hispavoces/find/master

You can download the last festvox-sflpc16k version .deb package and install with:
```bash
sudo dpkg -i festvox-sflpc16k_1.0-1_all.deb
```
You can test your installation with this command:
```bash
echo "Robocomp es el mejor software de robótica de Extremadura" | iconv -f utf-8 -t iso-8859-1 | festival --tts --language spanish
```

## Configuration parameters
As any other component,**`speech`** needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. You will have to set the Speech.Endpoint port to an available one in your deployment:

```bash
# Endpoints for implemented interfaces
Speech.Endpoints=tcp -p **10021**

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

# Component properties
#
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

    
## Starting the component
To avoid changing the **config** file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```bash
    cd  <speechComp 's path> 
    cp etc/config config
```

After editing the new config file we can run the component:
```bash
python2.7 src/speechComp.py config
```

## Testing this component
Robocomp have a tool to test some components: rcmonitor. Currently there's an example to test Speech Component. Tu run this test you need to configure the port you are running the component in the speech.rcm fine in  ~/robocomp/tools/rcmonitor/examples
```bash
/interfaces/Speech.ice
speech
localhost
tcp
10021
30
speech.py
```

As soon as you have this configured you can test it like this:

	cd ~/robocomp/tools/rcmonitor
	python2 rcmonitor.py examples/speech.rcm

The rcmonitor UI for this examples looks like this:

![Screenshot from 2019-05-06 13-15-51](https://user-images.githubusercontent.com/5784096/57222000-626b7b00-7001-11e9-86fe-06cf11207450.png)

You can use the **Random** button to talk some random sentence or use the input and click on **Say** button to send it to the Speech component.
