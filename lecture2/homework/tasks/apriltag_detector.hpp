#ifndef AUTO_CHARGE__APRILTAG_DETECTOR_HPP
#define AUTO_CHARGE__APRILTAG_DETECTOR_HPP

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "charge_types.hpp"

namespace auto_charge
{

class AprilTagDetector
{
public:
  explicit AprilTagDetector(const std::string & config_path);

  // 检测图像中的所有目标AprilTag
  std::vector<TagDetection> detect(const cv::Mat & img) const;

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::vector<int> target_ids_;
  double tag_size_ = 0.080;  // m，AprilTag 边长
};

}  // namespace auto_charge

#endif  // AUTO_CHARGE__APRILTAG_DETECTOR_HPP
