#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <optional>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

// 任务参数与限制
constexpr double FIRE_INTERVAL_S = 0.2;// 开火间隔：0.2秒(即5Hz弹频)
constexpr int    MAX_FIRE_COUNT  = 10;// 每个转速档位最多打10发    
constexpr double AIM_YAW_TOL     = 0.03;// 瞄准偏航角容差   
constexpr double AIM_PITCH_TOL   = 0.03;// 瞄准俯仰角容差
constexpr double PITCH_MAX       =  20.0 * CV_PI / 180.0;// 云台俯仰角上限
constexpr double PITCH_MIN       = -20.0 * CV_PI / 180.0;// 云台俯仰角下限

int main(int argc, char * argv[])
{
  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 创建OpenCV图像显示窗口
  cv::namedWindow("camera_view", cv::WINDOW_NORMAL);

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // 使用 std::optional，因为刚开机时没有目标，不能强行构造 Target 对象
  std::optional<auto_aim::Target> target;

  // 开火状态管理
  int fire_count = 0;
  auto last_fire_time = std::chrono::steady_clock::now() - 1s;
  int frame_count = 0;

  constexpr int TARGET_ARMOR_NUM = 4;

  while (!exiter.exit()) {
    // Your code start

    // 图像感知与状态读取
    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    auto armor_list = yolo.detect(img, frame_count++);

    std::optional<auto_aim::Armor> selected;
    for (auto & armor : armor_list) {
      if (!selected || armor.priority < selected->priority) selected = armor;
    }

    //EKF 目标跟踪与更新
    if (selected) {
      solver.solve(*selected);
      if (!target) {
        target.emplace(*selected, t);
      } else {
        target->predict(t);
        target->update(*selected);
      }
    } else if (target) {
      target->predict(t);  
    }

    //弹道解算与开火判断
    auto gimbal_state = gimbal.state();

    double cmd_yaw   = 0.0;
    double cmd_pitch = 0.0;
    double ekf_w     = 0.0;
    double ekf_angle = 0.0;
    bool   valid_target = false;
    bool   fire = false;

    if (target && target->isinit && target->is_converged()) {
      const auto & x = target->ekf_x();
      ekf_w     = x[7];   
      ekf_angle = x[6];
    
      //弹丸飞行时间 & 预测瞄准点
      double bullet_speed = gimbal_state.bullet_speed;
      if (bullet_speed < 1.0) bullet_speed = 25.0;

      double t_flight = 0.1;
      double pred_x = 0, pred_y = 0, pred_z = 0;
      for (int iter = 0; iter < 3; ++iter) {
        double cx = x[0] + x[1] * t_flight;   
        double cy = x[2] + x[3] * t_flight;   
        double cz = x[4] + x[5] * t_flight;   
        double a_base = tools::limit_rad(x[6] + x[7] * t_flight);
        double center_yaw = std::atan2(cy, cx);
        //寻找 t_flight 后最正对云台的装甲板 ID
        int best_id = 0;
        double min_angle_err = 1e10;
        for (int i = 0; i < TARGET_ARMOR_NUM; ++i) {
          double a_i = tools::limit_rad(a_base + i * 2 * CV_PI / TARGET_ARMOR_NUM);
          double angle_err = std::abs(tools::limit_rad(a_i - (center_yaw + CV_PI)));
          if (angle_err < min_angle_err) {
            min_angle_err = angle_err;
            best_id = i;
          }
        }

        bool use_l_h = (TARGET_ARMOR_NUM == 4) && (best_id == 1 || best_id == 3);
        double r  = use_l_h ? (x[8] + x[9]) : x[8];
        double dh = use_l_h ? x[10] : 0.0;

        double a_best = tools::limit_rad(a_base + best_id * 2 * CV_PI / TARGET_ARMOR_NUM);

        pred_x = cx - r * std::cos(a_best);
        pred_y = cy - r * std::sin(a_best);
        pred_z = cz + dh;

        double dist = std::sqrt(pred_x*pred_x + pred_y*pred_y + pred_z*pred_z);
        t_flight = dist / bullet_speed;
      }
      //计算云台偏航与俯仰角
      double dist_xy = std::sqrt(pred_x*pred_x + pred_y*pred_y);
      cmd_yaw   = std::atan2(pred_y, pred_x);
      cmd_pitch = -std::atan2(pred_z, dist_xy);
      cmd_pitch = tools::limit_min_max(cmd_pitch, PITCH_MIN, PITCH_MAX);

      valid_target = true;

      double yaw_err   = std::abs(tools::limit_rad(cmd_yaw - gimbal_state.yaw));
      double pitch_err = std::abs(cmd_pitch - gimbal_state.pitch);
      bool aim_ready = (yaw_err < AIM_YAW_TOL && pitch_err < AIM_PITCH_TOL);

      auto now = std::chrono::steady_clock::now();
      double dt_since_fire = std::chrono::duration<double>(now - last_fire_time).count();

      if (aim_ready && fire_count < MAX_FIRE_COUNT && dt_since_fire >= FIRE_INTERVAL_S) {
        fire = true;
        fire_count++;
        last_fire_time = now;
      }
    }

    
    if (valid_target) {
      gimbal.send(true, fire, static_cast<float>(cmd_yaw), static_cast<float>(cmd_pitch));
    } else {
      gimbal.send(false, false, 0.0f, 0.0f);
    }

    //可视化与数据输出
    cv::imshow("camera_view", img);
    int key = cv::waitKey(1);
    if (key == 'q') { exiter.exit(); break; }

    nlohmann::json j;
    j["gimbal_yaw_current"]  = gimbal_state.yaw;
    j["gimbal_pitch_current"]= gimbal_state.pitch;
    j["cmd_yaw"]             = cmd_yaw;    
    j["cmd_pitch"]           = cmd_pitch;  
    j["ekf_w"]               = ekf_w;      
    j["ekf_angle"]           = ekf_angle;
    j["bullet_count"]        = gimbal_state.bullet_count;
    j["fire_count"]          = fire_count;
    j["fire"]                = fire ? 1 : 0;

    if (target) {
      auto x = target->ekf_x();
      j["r"] = x[8];
      j["l"] = x[9];
      j["h"] = x[10];
    }
    plotter.plot(j);

    // Your code end
  }

  cv::destroyAllWindows();
  return 0;
}