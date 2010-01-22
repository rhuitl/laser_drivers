rosbuild_add_executable(print_scans print_scans.cpp)
target_link_libraries(pthread)

rosbuild_add_executable(time_scans time_scans.cpp)
target_link_libraries(pthread)

rosbuild_add_executable(log_scans log_scans.cpp)
target_link_libraries(pthread)
