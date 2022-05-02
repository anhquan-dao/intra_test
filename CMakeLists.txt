cmake_minimum_required(VERSION 2.8.3)
project(intra_test)

add_compile_options(-std=c++17 -rdynamic)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  nav_msgs
  std_msgs
  shm_transport
  moveit_ros_planning_interface
)
find_package(Boost REQUIRED COMPONENTS python)
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mycpplib2 SHMTransportCPP
  CATKIN_DEPENDS 
    roscpp
    rospy
    nav_msgs
    std_msgs
    shm_transport
    moveit_ros_planning_interface
  DEPENDS rt
)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(mycpplib2 SHARED 
  src/mycpplib.cpp
)

set_target_properties(mycpplib2 PROPERTIES 
  PREFIX "" 
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(mycpplib2
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
)

install(TARGETS mycpplib2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

add_library(SHMTransportCPP SHARED 
  src/shm_tp_lib_cpp.cpp
)

set_target_properties(SHMTransportCPP PROPERTIES 
  PREFIX "" 
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(SHMTransportCPP
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
  -lrt
)

install(TARGETS SHMTransportCPP
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

add_executable(map_shm_talker src/shm_test_talker.cpp)
target_link_libraries(map_shm_talker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lrt)
add_dependencies(map_shm_talker shm_transport_generate_messages_cpp)

add_executable(map_shm_listener src/shm_test_listener.cpp)
target_link_libraries(map_shm_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lrt)
add_dependencies(map_shm_listener shm_transport_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/use_resource_cpp.py scripts/shm_test_listener.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})