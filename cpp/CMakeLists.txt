# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(min_snap)

# Add the directory to search for header files
include_directories(include)

# Define 
file( GLOB ALGLIB_SOURCES include/ALGLIB/*.cpp )
file( GLOB ALGLIB_HEADERS include/ALGLIB/*.h )
file( GLOB MY_SOURCES include/*.cpp )
file( GLOB MY_HEADERS include/*.h )
add_executable(min_snap src/min_snap.cpp ${ALGLIB_SOURCES} ${ALGLIB_HEADERS} ${MY_SOURCES} ${MY_HEADERS})

