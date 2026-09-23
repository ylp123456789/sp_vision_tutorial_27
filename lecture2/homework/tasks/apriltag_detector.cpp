#include "apriltag_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace auto_charge
{
AprilTagDetector::AprilTagDetector(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto charge_yaml = yaml["auto_charge"];

  // 加载目标tag ID
  target_ids_ = tools::read_or<std::vector<int>>(charge_yaml, "tag_ids", {10, 18, 24});

  // 加载tag家族与边长
  auto tag_family = tools::read_or<std::string>(charge_yaml, "tag_family", "tag36h11");
  tag_size_ = tools::read_or<double>(charge_yaml, "tag_size", 0.080);

  // 初始化AprilTag字典
  if (tag_family == "tag36h11") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  } else if (tag_family == "tag25h9") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
  } else if (tag_family == "tag16h5") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  } else {
    tools::logger()->error("[auto_charge] Unsupported tag family: {}", tag_family);
    exit(1);
  }

  // 检测参数
  detector_params_ = cv::aruco::DetectorParameters::create();

  // 自适应阈值参数
  detector_params_->adaptiveThreshWinSizeMin =
    tools::read_or<int>(charge_yaml, "adaptive_thresh_win_size_min", 3);
  detector_params_->adaptiveThreshWinSizeMax =
    tools::read_or<int>(charge_yaml, "adaptive_thresh_win_size_max", 23);
  detector_params_->adaptiveThreshWinSizeStep =
    tools::read_or<int>(charge_yaml, "adaptive_thresh_win_size_step", 10);
  detector_params_->adaptiveThreshConstant =
    tools::read_or<double>(charge_yaml, "adaptive_thresh_constant", 7.0);

  // 最小周长
  detector_params_->minMarkerPerimeterRate =
    tools::read_or<double>(charge_yaml, "min_marker_perimeter_rate", 0.03);

  // 亚像素角点精化
  int corner_refine = tools::read_or<int>(charge_yaml, "corner_refinement_method", 1);
  detector_params_->cornerRefinementMethod = static_cast<cv::aruco::CornerRefineMethod>(corner_refine);

  tools::logger()->info(
    "[auto_charge] AprilTagDetector initialized: {}, size {:.3f}m, {} target IDs", tag_family,
    tag_size_, target_ids_.size());
}

std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat & img) const
{
  std::vector<TagDetection> results;

  if (img.empty()) return results;

  // 转灰度
  cv::Mat gray;
  if (img.channels() == 3)
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else
    gray = img;

  // 检测所有tag
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejected;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

  // 过滤目标ID
  for (size_t i = 0; i < ids.size(); i++) {
    if (std::find(target_ids_.begin(), target_ids_.end(), ids[i]) == target_ids_.end()) continue;

    TagDetection det;
    det.id = ids[i];
    det.corners = corners[i];

    // 计算中心点
    cv::Point2f center(0, 0);
    for (const auto & pt : corners[i]) {
      center.x += pt.x;
      center.y += pt.y;
    }
    center.x /= 4.0f;
    center.y /= 4.0f;
    det.center = center;

    results.push_back(std::move(det));
  }

  return results;
}

}  // namespace auto_charge
