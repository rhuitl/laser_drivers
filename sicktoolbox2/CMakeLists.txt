cmake_minimum_required(VERSION 2.8.3)
project(sicktoolbox2)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SickLD SickLMS1xx SickLMS2xx SickLMS5xx
  CATKIN_DEPENDS 
  DEPENDS Threads system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/sicktoolbox2 ${catkin_INCLUDE_DIRS})

# Driver libraries
add_library(SickLD src/drivers/ld/sickld/SickLD.cc src/drivers/ld/sickld/SickLDBufferMonitor.cc src/drivers/ld/sickld/SickLDMessage.cc)
target_link_libraries(SickLD ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS1xx src/drivers/lms1xx/sicklms1xx/SickLMS1xx.cc src/drivers/lms1xx/sicklms1xx/SickLMS1xxBufferMonitor.cc src/drivers/lms1xx/sicklms1xx/SickLMS1xxMessage.cc)
target_link_libraries(SickLMS1xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS2xx src/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc src/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc src/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc)
target_link_libraries(SickLMS2xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS5xx src/drivers/lms5xx/sicklms5xx/SickLMS5xx.cc src/drivers/lms5xx/sicklms5xx/SickLMS5xxBufferMonitor.cc src/drivers/lms5xx/sicklms5xx/SickLMS5xxMessage.cc)
target_link_libraries(SickLMS5xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS SickLD SickLMS1xx SickLMS2xx SickLMS5xx
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
