

set (CMAKE_CXX_FLAGS "-mmmx -msse -msse2 -O3")
set(CMAKE_C_FLAGS "-Wall -O3 -g -save-temps")

find_package(Threads REQUIRED)
find_package(USB REQUIRED)

find_library (PTHREAD pthread)

include_directories(${PROJECT_SOURCE_DIR}/src/kinect/include)
include_directories(${PROJECT_SOURCE_DIR}/src/kinect/lib)
include_directories(${USB_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR}/include)
include_directories (${CMAKE_SOURCE_DIR}/lib)

SET(SRC motors.c accelerometers.c core.c cameras.c usb_libusb10.c)

add_library (lib_freenect SHARED ${SRC} ${CMAKE_CURRENT_BINARY_DIR}/inits.c)
target_link_libraries (lib_freenect ${CMAKE_THREAD_LIBS_INIT} ${USB_LIBRARIES})

add_library (lib_freenectstatic STATIC ${SRC} ${CMAKE_CURRENT_BINARY_DIR}/inits.c)
set_target_properties (lib_freenectstatic PROPERTIES OUTPUT_NAME freenect)
target_link_libraries (lib_freenectstatic ${CMAKE_THREAD_LIBS_INIT} ${USB_LIBRARIES})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/inits.c
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/geninits.py
    MAIN_DEPENDENCY inits.txt
    COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/geninits.py ${CMAKE_CURRENT_SOURCE_DIR}/inits.txt ${CMAKE_CURRENT_BINARY_DIR}/inits.c)

