cmake_minimum_required(VERSION 2.8.3)
project(flight_test)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  #INCLUDE_DIRS include
  INCLUDE_DIRS /usr/local/include
  DEPENDS CppLinuxSerial
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(flight_test flight_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(flight_test
  ${catkin_LIBRARIES}
)

add_executable(cmd_hold cmd_hold.cpp)
target_link_libraries(cmd_hold
  ${catkin_LIBRARIES}
)

add_executable(cmd_stop_serial cmd_stop_serial.cpp)
target_link_libraries(cmd_stop_serial
  ${catkin_LIBRARIES}
  CppLinuxSerial
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS hello_cpp hello_cpp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
