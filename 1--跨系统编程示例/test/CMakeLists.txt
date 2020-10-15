cmake_minimum_required(VERSION 3.12)

project(test)

# use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)


# user program started...
set(SOURCE_FILES src/main.cpp)
add_executable(test ${SOURCE_FILES})
if(UNIX)
	target_link_libraries(test ${codeit_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(test ${codeit_LIBRARIES})
endif(WIN32)
