#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

#include <nlohmann/json.hpp>

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  double last_target_yaw = 0.0;
  double last_target_pitch = 0.0;


  while (!exiter.exit()) {
    // Your code start
    // 1. 读取图像 + 获取帧时间戳
    camera.read(img, t);

    // 2. 根据当前帧时间戳，插值获取云台IMU四元数（核心！四元数处理）
    q = gimbal.q(t);
    // 3. 将四元数传入solver，更新云台到世界的旋转矩阵，用于PnP解算
    solver.set_R_gimbal2world(q);

    // 4. YOLO推理，识别画面内所有装甲板
    std::list<auto_aim::Armor> armor_list = yolo.detect(img);

    double target_yaw = last_target_yaw;
    double target_pitch = last_target_pitch;

    // 如果检测到装甲
    if (!armor_list.empty())
    {
      // 取第一个装甲板（任务一简单跟随，不做多装甲选择）
      auto_aim::Armor armor = armor_list.front();
      // 5. PnP解算：像素点 -> 相机坐标系 -> 云台坐标系 -> 世界坐标系xyz
      solver.solve(armor);

      // armor.ypd_in_world = [yaw, pitch, distance]，单位rad
      target_yaw = armor.ypd_in_world[0];
      target_pitch = armor.ypd_in_world[1];

      // 角度限幅：题目要求pitch ±20°
      const double pitch_max = 20.0 * CV_PI / 180.0;
      const double pitch_min = -20.0 * CV_PI / 180.0;
      target_pitch = tools::limit_min_max(target_pitch, pitch_min, pitch_max);

      // 更新上一帧角度
      last_target_yaw = target_yaw;
      last_target_pitch = target_pitch;
    }

    // 6. Plotter记录曲线（考核截图用：目标yaw/pitch，当前云台yaw/pitch）
    auto gimbal_state = gimbal.state();

    // 引入json头文件，如果没有的话需要加上 #include <nlohmann/json.hpp>
    nlohmann::json j;
    j["gimbal_yaw_current"] = gimbal_state.yaw;
    j["gimbal_pitch_current"] = gimbal_state.pitch;
    j["target_yaw"] = target_yaw;
    j["target_pitch"] = target_pitch;
    plotter.plot(j);


    // 7. 下发云台指令：control=true，不开火fire=false
    gimbal.send(true, false, static_cast<float>(target_yaw), static_cast<float>(target_pitch));

    // Your code end
  }

  return 0;
}