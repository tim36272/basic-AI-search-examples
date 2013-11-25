cmake_minimum_required(VERSION 2.8)
project( Assignment1 )

find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable( driver driver.cpp )
target_link_libraries( driver ${OpenCV_LIBS} )

