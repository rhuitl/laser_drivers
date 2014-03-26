add_executable(print_scans print_scans.cpp)
target_link_libraries(print_scans pthread)

add_executable(time_scans time_scans.cpp)
target_link_libraries(time_scans pthread)

add_executable(log_scans log_scans.cpp)
target_link_libraries(log_scans pthread)

install(TARGETS print_scans time_scans log_scans
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
