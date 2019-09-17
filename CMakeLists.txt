cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

find_package( catkin REQUIRED COMPONENTS 
    roscpp 
)

find_package( zeabus_library REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp zeabus_library
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
)
