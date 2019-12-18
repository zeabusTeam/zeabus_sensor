cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

find_package( catkin REQUIRED COMPONENTS 
    roscpp 
    zeabus_utility
)

find_package( zeabus_library REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp zeabus_library
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
)

add_executable( capture_gyro_bias src/imu_capture_gyro_bias.cpp)
target_link_libraries( capture_gyro_bias 
    ${zeabus_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( imu_cf src/imu_cf.cpp )
target_link_libraries( imu_cf 
    ${zeabus_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( imu_ahrs src/imu_ahrs.cpp )
target_link_libraries( imu_ahrs
    ${zeabus_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( pressure_buffer src/pressure_buffer.cpp )
target_link_libraries( pressure_buffer 
    ${catkin_LIBRARIES}
    ${zeabus_library_LIBRARIES}
)

add_executable( pressure_origin src/pressure_generate.cpp )
target_link_libraries( pressure_origin 
    ${catkin_LIBRARIES}
    ${zeabus_library_LIBRARIES}
)
