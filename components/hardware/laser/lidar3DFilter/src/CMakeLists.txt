# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/urdf_mesh_loader.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)


LIST(APPEND LIBS embree4 assimp tinyxml2)

# 1. Flags de Compilación (Cómo se genera el código)
add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
    -fopenmp
)

# 2. Definiciones Globales
add_definitions(-DNDEBUG -fmax-errors=1)

# 3. Flags de Enlace (Cómo se une todo)
# Quitamos -fuse-ld=gold para evitar el error de QGLViewer
add_link_options(-flto=auto -fopenmp)