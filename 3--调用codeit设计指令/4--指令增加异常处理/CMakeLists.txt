cmake_minimum_required(VERSION 3.12)

project(myplan)

if(WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif(WIN32)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find codeit
if(UNIX)
	add_definitions(-DUNIX)
	find_package(codeit REQUIRED PATHS /usr/codeit)
endif(UNIX)
IF(WIN32)
	find_package(codeit REQUIRED PATHS C:/codeit/codeit-1.0.0)
endif(WIN32)
include_directories(${codeit_INCLUDE_DIRS})

# user program started...
set(SOURCE_FILES src/main.cpp src/basicsystem.hpp src/basicsystem.cpp src/xfunc.hpp src/xfunc.cpp)
add_executable(myplan ${SOURCE_FILES})
target_link_libraries(myplan ${codeit_LIBRARIES})