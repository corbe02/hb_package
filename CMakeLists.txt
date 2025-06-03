cmake_minimum_required(VERSION 3.0.2)
project(hb_package)

## Abilita C++11
add_compile_options(-std=c++11)

## Trova pacchetti ROS richiesti
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


###################################
## Configurazione del pacchetto  ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Crea l'eseguibile C++ (main.cpp e particle_filter.cpp)
add_executable(particle_filter_node
  src/main.cpp
  src/particle_filter.cpp
  src/utils.cpp
)

target_link_libraries(particle_filter_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Installa gli script Python
catkin_install_python(PROGRAMS
  scripts/ecg_plotter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Installa il nodo C++
install(TARGETS particle_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
