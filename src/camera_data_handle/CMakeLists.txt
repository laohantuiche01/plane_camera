cmake_minimum_required(VERSION 3.10)
project(camera_data_handle)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenVINO REQUIRED)
ament_auto_find_build_dependencies()

#option(IF_USE_OPENVINO "Detection use openvino" ON)

#if(IF_USE_OPENVINO)
#    add_definitions(-DETECTION_OPENVINO_OPEN)
#    message(STATUS "OpenVINO support enabled-----------------------")
#else ()
#    message(STATUS "OpenVINO support unabled-----------------------!!!!!!!!!!")
#endif ()

#add_definitions(-DFPS_VISABLE_OPEN)
add_definitions(-DDETECTION_OPENVINO_OPEN)
add_definitions(-DKEY_POINT_TRACKING)
add_definitions(-DPREDICT_OPEN)
add_definitions(-DPROJECT_PATH=${CMAKE_CURRENT_SOURCE_DIR})

function(add_and_install_executable target_name)
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs SRCS DEPS LIBS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${target_name} ${ARG_SRCS})

    if (ARG_DEPS)
        ament_target_dependencies(${target_name} ${ARG_DEPS})
    endif ()

    if (ARG_LIBS)
        target_link_libraries(${target_name} ${ARG_LIBS})
    endif ()

    install(TARGETS ${target_name}
            DESTINATION lib/${PROJECT_NAME})
endfunction()

add_executable(linshi
        src/linshi.cpp
)
ament_target_dependencies(
        linshi
        OpenCV
)
install(TARGETS
        linshi
        DESTINATION lib/${PROJECT_NAME})

add_executable(receive_data
        src/receive_data.cpp
        src/Yolov8Detector.cpp
        src/camera_handle.cpp
        src/point_tracking.cpp
)
ament_target_dependencies(
        receive_data
        rclcpp
        OpenCV
        cv_bridge
        tf2
        tf2_ros
        tf2_geometry_msgs
        yaml-cpp

)
target_link_libraries(receive_data
        openvino::runtime
)
install(TARGETS
        receive_data
        DESTINATION lib/${PROJECT_NAME})

add_executable(tf_publish
        src/tf_publisher.cpp
)
ament_target_dependencies(
        tf_publish
        rclcpp
        tf2
        tf2_ros
        tf2_geometry_msgs
)
install(TARGETS
        tf_publish
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
)
