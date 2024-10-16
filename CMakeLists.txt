cmake_minimum_required(VERSION 3.10)

# Set the project name
project(Chirp)

# Add the executable
add_executable(chirp src/chirp.cpp)

# Include Directories
include_directories(${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/invn/common)

# Link the libraries
target_link_directories(chirp PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(chirp PRIVATE InvnAlgoRangeFinder m)
