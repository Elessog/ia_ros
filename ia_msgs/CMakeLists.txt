cmake_minimum_required(VERSION 2.8.3)
project(ia_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  message_runtime
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Interv.msg
   Interv3D.msg
   Interval.msg
   Interval3D.msg
   BeaconDist.msg
   DistInterBeacon.msg
   IdInterval.msg
   StampedInterval.msg
 )

add_service_files(
 FILES
 Start_Slam.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
  ia_msgs   geometry_msgs   std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ia_msgs
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



