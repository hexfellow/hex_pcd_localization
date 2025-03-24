cmake_minimum_required(VERSION 3.0.2)
project(hex_pcd_localization)

set(HEX_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${HEX_ROS_VERSION}")

add_definitions(-DHEX_ROS_VERSION=${HEX_ROS_VERSION}) 

if (${HEX_ROS_VERSION} STREQUAL "1")

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        pcl_ros
        pcl_conversions
    )
    find_package(Eigen3 REQUIRED)
    find_package(PCL REQUIRED)
    catkin_package()

    include_directories(
        ../
        ./
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS}
    )

    add_executable(hex_pcd_localization
        hex_pcd_localization/main.cc
        hex_pcd_localization/pcd_localization.cc
        hex_pcd_localization/data_interface/ros1_interface.cc
    )

    target_link_libraries(hex_pcd_localization
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
    )

    install(
        TARGETS hex_pcd_localization
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros1
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros1
        DESTINATION share/${PROJECT_NAME}/config
    )

elseif (${HEX_ROS_VERSION} STREQUAL "2")
    message(FATAL_ERROR "This package does not support ROS2")

    # cmake_minimum_required(VERSION 3.8)
    
    # if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #     add_compile_options(-Wall -Wextra -Wpedantic)
    # endif()

    # find_package(ament_cmake REQUIRED)
    # find_package(rclcpp REQUIRED)
    # find_package(std_msgs REQUIRED)

    # include_directories(
    #     ../
    #     ./
    #     include
    #     ${colcon_INCLUDE_DIRS}
    # )

    # ament_package()

    # add_executable(hex_pcd_localization
    #     hex_pcd_localization/main.cc
    #     hex_pcd_localization/hex_pcd_localization.cc
    #     hex_pcd_localization/data_interface/ros2_interface.cc
    # )

    # ament_target_dependencies(hex_pcd_localization
    #     rclcpp
    #     std_msgs
    # )

    # install(
    #     TARGETS hex_pcd_localization
    #     DESTINATION lib/${PROJECT_NAME}
    # )

    # install(
    #     DIRECTORY launch/ros2
    #     DESTINATION share/${PROJECT_NAME}/launch
    # )

    # install(
    #     DIRECTORY config/ros2
    #     DESTINATION share/${PROJECT_NAME}/config
    # )

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
