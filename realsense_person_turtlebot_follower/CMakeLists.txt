cmake_minimum_required(VERSION 2.8.3)
project(realsense_person_turtlebot_follower)


# Set compile flags
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
# Flags executables
set(CMAKE_EXE_LINKER_FLAGS "-pie -z noexecstack -z relro -z now")
# Flags shared libraries
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now")


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS nodelet roscpp dynamic_reconfigure realsense_person)
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/RealSensePersonCoM.cfg
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp dynamic_reconfigure realsense_person
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(
  ${PROJECT_NAME} src/realsense_person_com.cpp
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other required files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
