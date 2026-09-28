#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"
class Camera
{
public:
    Camera();
    ~Camera();
    bool read(cv::Mat& img);
    bool is_open_;
private:
    void* handle_;
    cv::Mat transfer(MV_FRAME_OUT& raw);
};

#endif
