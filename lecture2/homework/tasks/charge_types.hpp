#ifndef AUTO_CHARGE__CHARGE_TYPES_HPP
#define AUTO_CHARGE__CHARGE_TYPES_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

namespace auto_charge
{

// 单个AprilTag的检测结果
struct TagDetection
{
  int id;
  std::vector<cv::Point2f> corners;  // 图像坐标系下4个角点（左上→左下→右下→右上）
  cv::Point2f center;
};

}  // namespace auto_charge

#endif  // AUTO_CHARGE__CHARGE_TYPES_HPP
