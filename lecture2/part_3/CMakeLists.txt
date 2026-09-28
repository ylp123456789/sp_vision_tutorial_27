cmake_minimum_required(VERSION 3.16)

project(PART3_OPENCV)

# 告诉 CMake 我们需要 OpenCV，找不到会直接报错
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ex1：Hello OpenCV —— 读取并显示图片
add_executable(opencv_ex1 ex1/example1.cpp)
target_link_libraries(opencv_ex1 ${OpenCV_LIBS})

# ex2：图像就是 Mat 类的对象
add_executable(opencv_ex2 ex2/example2.cpp)
target_link_libraries(opencv_ex2 ${OpenCV_LIBS})

# 作业：TODO 由你自己完成
add_executable(homework homework/homework.cpp)
target_link_libraries(homework ${OpenCV_LIBS})