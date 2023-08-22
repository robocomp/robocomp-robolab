# sbg_ekinox

instala libreria https://github.com/SBG-Systems/sbgECom
- ubicamos en la carpera de software
- descargamos repo con  ``git add https://github.com/SBG-Systems/sbgECom.git``
- compilamos con ```cmake -Bbuild -DBUILD_EXAMPLES=ON -DBUILD_TOOLS=ON && cmake --build build```
- instalamos con ``sudo make install``

en el caso de que no funcione con el ```sudo make install``` existe otra opcion mas chapuza:

en el componente realiza el siguiente comando ``mkdir lib``
una vez compilado mete el build/libsbgECom.a del repo en lib/ del componente
realiza los siguente comando ``mkdir include && find src -name "*.h" -exec cp --parents \{} include/ \; && find common -name "*.h" -exec cp --parents \{} include/ \;``
este genera una carpeta include con todos los .h necesarios, copiamos esta carpeta include en al raiz del componente
introducir en en CMakeList.txt las lineas
```
link_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include/src ${CMAKE_SOURCE_DIR}/include/common)
```

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
