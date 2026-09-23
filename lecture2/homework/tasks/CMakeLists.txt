cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(tasks OBJECT
    armor.cpp
    yolo.cpp
    yolos/yolov5.cpp
    apriltag_detector.cpp
)

target_link_libraries(tasks io openvino::runtime )