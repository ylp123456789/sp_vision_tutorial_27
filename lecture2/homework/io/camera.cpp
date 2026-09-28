#include "camera.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include <unordered_map>
#include <stdexcept>

cv::Mat Camera::transfer(MV_FRAME_OUT& raw)
{
    int w = raw.stFrameInfo.nWidth;
    int h = raw.stFrameInfo.nHeight;
    cv::Mat bayer_img(cv::Size(w, h), CV_8UC1);
    memcpy(bayer_img.data, raw.pBufAddr, raw.stFrameInfo.nFrameLen);
    static const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}
    };
    auto pixel_type = raw.stFrameInfo.enPixelType;
    printf("Camera raw pixel type: %ld\n", pixel_type);
    auto it = type_map.find(pixel_type);
    if(it == type_map.end())
    {
        return cv::Mat();
    }
    cv::Mat bgr_img;
    cv::cvtColor(bayer_img, bgr_img, it->second);
    cv::cvtColor(bgr_img, bgr_img, cv::COLOR_RGB2BGR);
    return bgr_img;
}

Camera::Camera() : is_open_(false), handle_(nullptr)
{
    int ret;
    MV_CC_DEVICE_INFO_LIST device_list;
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        return;
    }
    if (device_list.nDeviceNum == 0) {
        return;
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        handle_ = nullptr;
        return;
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 2000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret == MV_OK)
    {
        is_open_ = true;
    }
    else
    {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
}

bool Camera::read(cv::Mat& img)
{
    if(!is_open_ || handle_ == nullptr)
        return false;

    int ret;
    MV_FRAME_OUT raw;
    unsigned int timeout_ms = 100;
    ret = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms);
    if (ret != MV_OK)
    {
        return false;
    }
    img = transfer(raw);

    ret = MV_CC_FreeImageBuffer(handle_, &raw);
    if (ret != MV_OK)
    {
        return false;
    }
    return !img.empty();
}

Camera::~Camera()
{
    if (!is_open_ || handle_ == nullptr)
        return;

    int ret;
    ret = MV_CC_StopGrabbing(handle_);
    ret = MV_CC_CloseDevice(handle_);
    ret = MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    is_open_ = false;
}

