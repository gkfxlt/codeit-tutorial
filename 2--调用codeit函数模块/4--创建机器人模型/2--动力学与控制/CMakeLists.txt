cmake_minimum_required(VERSION 3.12)

project(codeit-deploy)

if(WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif(WIN32)

# codeit use c++ 17 standard
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

message(${codeit_DIR})

# user program started...
set(SOURCE_FILES src/main.cpp src/basicsystem.hpp src/basicsystem.cpp)
add_executable(codeit-deploy ${SOURCE_FILES})
if(UNIX)
	target_link_libraries(codeit-deploy ${codeit_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(codeit-deploy ${codeit_LIBRARIES})
endif(WIN32)
