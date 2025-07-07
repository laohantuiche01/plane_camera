cmake_minimum_required(VERSION 3.8)
project(camera_data_handle)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(ONNX_RUNTIME_DIR "/home/zxk/桌面/original_data/onnxruntime/onnxruntime")
find_library(ONNX_RUNTIME_LIB
        NAMES onnxruntime
        HINTS ${ONNX_RUNTIME_DIR}/build/Linux/Release
)
find_path(ONNX_RUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        HINTS ${ONNX_RUNTIME_DIR}/include/onnxruntime/core/session
)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()


add_executable(receive_data
        src/receive_data.cpp
        src/Yolov8Detector.cpp
        src/camera_handle.cpp
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
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
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
