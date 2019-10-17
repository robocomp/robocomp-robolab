# speechComp

The `speechComp` component is a Robocomp Component wrapper implementation of several libraries for the Text-to-Speech(TTS) task. The underlying logic is to run bash command utilizing `festival` TTS library or online TTS Google service for different languages via Python environment. This component implements the `Speech.idsl` interface to receive string arguments to be spoken in the TTS.

## Resolving dependencies

This section assumes the user has already installed the RoboComp core library and pulled Robolab's components according to this [README guide](https://github.com/robocomp/robocomp). There are two TTS services:

### festival

The current implementation provides two languages for TTS:

- **English**
To use English TTS, the user must install `festival` library, which is already available in Ubuntu PPA:

```bash
sudo apt install festival
```

- **Spanish**

The default language of `festival` is English. If the user would like Spanish TTS, the user can download the Junta de Andalucía's synthetic voices that can be found in this [Github repository](https://github.com/guadalinex-archive/hispavoces/find/master).

The user should download the latest `festvox-sflpc16k_1.0-1_all.deb` package and install with:
```bash
sudo dpkg -i festvox-sflpc16k_1.0-1_all.deb
```

and test the installation with this command:
```bash
echo "Robocomp es el mejor software de robótica de Extremadura" | iconv -f utf-8 -t iso-8859-1 | festival --tts --language spanish
```

### Google TTS
For the online Google TTS service, there are many more languages to choose from. Please install the `gTTS` library:

```
pip install gTTS
```

After installing all dependencies, we can start using this component.

## Configuration parameters
`speechComp` component parameters are characterized as the `etc/config` file described below:

```
# Endpoints for implemented interfaces
Speech.Endpoints=tcp -p 10021

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

tts=google
# Component properties
#
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

By setting `tts` parameter, we can switch between TTS services of Google and `festival` (e.g `tts=google` for Google TTS and `tts=festival` for `festival` library).
Note that we need to make sure the port number of the parameter `Speech.Endpoints` is the same as the corresponding number of the client component using the `speechComp` component.


## Starting the component
To avoid changing the config file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:
```
cp etc/config etc/config-run
```

After editing the new config file we can run the component:
```
python src/speechComp.py etc/config-run
```

## Testing this component

We will use `rcmonitor` to test this component. Currently, there is an example to test `speechComp` specified in file `~/robocomp/tools/rcmonitor/examples/speech.rcm`. A snippet of `speech.rcm` is shown below:

```
/interfaces/Speech.ice
speech
localhost
tcp
10021
30
speech.py
```

We can test by:

```
cd ~/robocomp/tools/rcmonitor
python rcmonitor.py examples/speech.rcm
```

The **rcmonitor UI** for this examples looks like this:

![Screenshot from 2019-05-06 13-15-51](https://user-images.githubusercontent.com/5784096/57222000-626b7b00-7001-11e9-86fe-06cf11207450.png)

You can use the **Random** button to talk some random sentences or use the input field form, then click on **Say** button to send it to the `speechComp`.
