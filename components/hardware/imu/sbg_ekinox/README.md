# sbg_ekinox


instala libreria https://github.com/SBG-Systems/sbgECom
mete el build/libsbgECom.a del repo en el lib/ del componente
realiza los siguente comando 

``mkdir include && find src -name "*.h" -exec cp --parents \{} include/ \; && find common -name "*.h" -exec cp --parents \{} include/ \;``
este genera una carpeta include con todos los .h necesarios, copiamos el contenido de la carpeta en include/ del componente

Intro to component here


## Configuration parameters
As any other component, *sbg_ekinox* needs a configuration file to start. In
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
cd <sbg_ekinox's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/sbg_ekinox config
```
