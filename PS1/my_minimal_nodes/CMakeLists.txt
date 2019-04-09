cmake_minimum_required(VERSION 2.8.3)
project(my_minimal_nodes)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)

add_executable(my_minimal_publisher src/minimal_publisher.cpp)
add_executable(my_sleepy_minimal_publisher src/sleepy_minimal_publisher.cpp)
add_executable(my_minimal_subscriber src/minimal_subscriber.cpp)
add_executable(my_minimal_simulator src/minimal_simulator.cpp)
add_executable(my_minimal_controller src/minimal_controller.cpp)
add_executable(my_sin_commander src/sin_commander.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)

target_link_libraries(my_minimal_publisher ${catkin_LIBRARIES})
target_link_libraries(my_sleepy_minimal_publisher ${catkin_LIBRARIES})
target_link_libraries(my_minimal_subscriber ${catkin_LIBRARIES})
target_link_libraries(my_minimal_simulator ${catkin_LIBRARIES})
target_link_libraries(my_minimal_controller ${catkin_LIBRARIES})
target_link_libraries(my_sin_commander ${catkin_LIBRARIES})

cs_install()
cs_export()
    
