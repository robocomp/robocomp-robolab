```
```
#
``` CGR
```
Intro to component here


## Configuration parameters
As any other component,
``` *CGR* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <CGR 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```CGR ```

    --Ice.Config=config
For install LUA:
 sudo apt-get install sudo apt-get install lua5.1 && liblua5.1-0-dev
 sudo ln -s /usr/lib/x86_64-linux-gnu/lublua5.1.so /usr/lib/liblua5.1.so 
 sudo ln -s /usr/include/lua5.1/lua.h /usr/include/lua.h
 sudo ln -s /usr/include/lua5.1/lualib.h /usr/include/lualib.h
 sudo ln -s /usr/include/lua5.1/luaconf.h /usr/include/luaconf.h
 Add to CMakeListsSpecific.txt 
 	
 find_package(Lua51 REQUIRED)
 set (SPECIFIC_LIBS ${LUA_LIBRARIES} )
