add_executable(sicklms_1xx sicklms1xx.cpp)
add_executable(sicklms_2xx sicklms2xx.cpp)
add_executable(sicklms_5xx sicklms5xx.cpp)

install(TARGETS sicklms_1xx sicklms_2xx sicklms_5xx
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
