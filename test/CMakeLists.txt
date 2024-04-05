cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-variable")
add_executable(MyProject main.cpp)

include_directories(
	/usr/lib/
	./inc/
)

find_library(TDK_CHIRP_LIB
	NAMES tdk-chx01-get-data
	PATHS /usr/lib
)

# Assume that your shared library is named `libtdk-chx01-get-data.so`
target_link_libraries(MyProject tdk-chx01-get-data)




