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
#include "tools/trajectory.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

// ===================== 任务参数与限制 =====================
constexpr double FIRE_INTERVAL_S     = 1.5;                    // 开火间隔：0.2s (5Hz)
constexpr int    MAX_FIRE_COUNT      = 10;                     // 每档最多 10 发
constexpr double AIM_YAW_TOL         = 0.03;                   // 瞄准偏航容差 (rad)
constexpr double AIM_PITCH_TOL       = 0.03;                   // 瞄准俯仰容差 (rad)
constexpr double PITCH_MAX           =  20.0 * CV_PI / 180.0;  // 云台俯仰上限
constexpr double PITCH_MIN           = -20.0 * CV_PI / 180.0;  // 云台俯仰下限
constexpr double SYS_DELAY_AFTER_NOW = 0.03;                   // now → 出膛的固定延时
constexpr int    TARGET_ARMOR_NUM    = 4;                      // 4 块装甲板
constexpr double SWITCH_HYST         = 0.10;                   // 滞回切换阈值 (rad)
constexpr double FIRE_ALIGN_TOL      = 0.35;                   // 命中时刻板对齐阈值 (rad，≈20°)
// ==========================================================

int main(int argc, char * argv[])
{
  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  cv::namedWindow("camera_view", cv::WINDOW_NORMAL);

  tools::Exiter  exiter;
  tools::Plotter plotter;

  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  auto_aim::YOLO   yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t_img;

  std::optional<auto_aim::Target> target;

  int  fire_count     = 0;
  auto last_fire_time = std::chrono::steady_clock::now() - 1s;
  int  frame_count    = 0;

  // 跨帧保存的状态（放到 while 外）
  int tracked_id    = -1;
  int aim_ready_cnt = 0;

  while (!exiter.exit()) {
    // ---------- 图像感知与状态读取 ----------
    camera.read(img, t_img);
    q = gimbal.q(t_img);
    solver.set_R_gimbal2world(q);

    auto armor_list = yolo.detect(img, frame_count++);

    std::optional<auto_aim::Armor> selected;
    for (auto & armor : armor_list) {
      if (!selected || armor.priority < selected->priority) selected = armor;
    }

    if (selected) {
      solver.solve(*selected);
      if (!target) {
        target.emplace(*selected, t_img);
      } else {
        target->predict(t_img);
        target->update(*selected);
      }
    } else if (target) {
      target->predict(t_img);
    }

    auto gimbal_state = gimbal.state();

    // ---------- 弹道解算与开火判断 ----------
    double cmd_yaw          = 0.0;
    double cmd_pitch        = 0.0;
    double ekf_w            = 0.0;
    double ekf_angle        = 0.0;
    double t_process        = 0.0;
    double t_flight         = 0.1;
    double err_at_hit       = 1e10;
    bool   aligned_to_armor = false;
    bool   valid_target     = false;
    bool   fire             = false;
    double yaw_err          = 1e10;
    double pitch_err        = 1e10;
    bool   aim_ready        = false;

    if (target && target->isinit && target->is_converged() && !target->diverged()) {
      const auto & x = target->ekf_x();
      ekf_w     = x[7];
      ekf_angle = x[6];

      double bullet_speed = gimbal_state.bullet_speed;
      if (bullet_speed < 1.0) bullet_speed = 25.0;

      // 处理延时（YOLO 推理 + 解算 + 上一帧相机采集结束到现在）
      auto t_pred = std::chrono::steady_clock::now();
      t_process   = std::chrono::duration<double>(t_pred - t_img).count();

      double pred_x = 0, pred_y = 0, pred_z = 0;
      double t_flight_local = t_flight;
      double L_x = 0.171, L_y = 0.0;

      for (int iter = 0; iter < 3; ++iter) {
        double total_t = t_process + SYS_DELAY_AFTER_NOW + t_flight_local;

        double cx = x[0];
        double cy = x[2];
        double cz = x[4];
        double a_base     = tools::limit_rad(x[6] + x[7] * total_t);
        double center_yaw = std::atan2(cy, cx);

        // 计算每块板在命中时刻的正对误差
        double err_arr[TARGET_ARMOR_NUM];
        int    new_best     = 0;
        double new_best_err = 1e10;
        for (int i = 0; i < TARGET_ARMOR_NUM; ++i) {
          double a_i = tools::limit_rad(a_base + i * 2 * CV_PI / TARGET_ARMOR_NUM);
          err_arr[i] = std::abs(tools::limit_rad(a_i - (center_yaw + CV_PI)));
          if (err_arr[i] < new_best_err) {
            new_best_err = err_arr[i];
            new_best     = i;
          }
        }

        // 滞回：仅在 iter == 0 决策一次，避免 3 轮迭代里反复切换
        if (iter == 0) {
          if (tracked_id < 0) {
            tracked_id = new_best;
          } else if (new_best_err + SWITCH_HYST < err_arr[tracked_id]) {
            tracked_id = new_best;
          }
        }

        bool   use_l_h = (tracked_id == 1 || tracked_id == 3);
        double r       = use_l_h ? (x[8] + x[9]) : x[8];
        double dh      = use_l_h ? x[10] : 0.0;
        double a_best  = tools::limit_rad(a_base + tracked_id * 2 * CV_PI / TARGET_ARMOR_NUM);

        pred_x = cx - r * std::cos(a_best);
        pred_y = cy - r * std::sin(a_best);
        pred_z = cz + dh;

        // 命中时刻锁定板的对齐误差（供开火门控）
        err_at_hit = std::abs(tools::limit_rad(a_best - (center_yaw + CV_PI)));

        double d_center = std::sqrt(pred_x * pred_x + pred_y * pred_y);
        double h_center = pred_z;

        double final_pitch_positive_up = 0.0;
        double final_fly_time          = 0.0;
        if (tools::solve_actual_trajectory(bullet_speed, d_center, h_center, 3, L_x, L_y,
              final_pitch_positive_up, final_fly_time)) {
          t_flight_local = final_fly_time;
          cmd_pitch      = -final_pitch_positive_up;
        } else {
          cmd_pitch      = -std::atan2(h_center, d_center);
          t_flight_local = d_center / bullet_speed;
        }
      }
      t_flight = t_flight_local;

      cmd_yaw      = std::atan2(pred_y, pred_x);
      cmd_pitch    = tools::limit_min_max(cmd_pitch, PITCH_MIN, PITCH_MAX);
      valid_target = true;

      yaw_err   = std::abs(tools::limit_rad(cmd_yaw - gimbal_state.yaw));
      pitch_err = std::abs(cmd_pitch - gimbal_state.pitch);
      aim_ready = (yaw_err < AIM_YAW_TOL && pitch_err < AIM_PITCH_TOL);

      // 命中时刻锁定的板必须正对云台，才允许开火
      aligned_to_armor = (err_at_hit < FIRE_ALIGN_TOL);

      aim_ready_cnt =
        (aim_ready && aligned_to_armor) ? std::min(aim_ready_cnt + 1, 10) : 0;

      auto   now           = std::chrono::steady_clock::now();
      double dt_since_fire = std::chrono::duration<double>(now - last_fire_time).count();
      if (aim_ready_cnt >= 3 && dt_since_fire >= FIRE_INTERVAL_S )
      {
        fire           = true;
        fire_count++;
        last_fire_time = now;
      }
    } else {
      tracked_id    = -1;
      aim_ready_cnt = 0;
    }

    // ---------- 云台下发 ----------
    if (valid_target) {
      gimbal.send(true, fire, static_cast<float>(cmd_yaw), static_cast<float>(cmd_pitch));
    } else {
      gimbal.send(false, false, static_cast<float>(gimbal_state.yaw), static_cast<float>(gimbal_state.pitch));
    }

    // ---------- 可视化 ----------
    cv::imshow("camera_view", img);
    int key = cv::waitKey(1);
    if (key == 'q') { exiter.exit(); break; }

    // ---------- Plotter 输出 ----------
    nlohmann::json j;
    j["gimbal_yaw_current"]   = gimbal_state.yaw;
    j["gimbal_pitch_current"] = gimbal_state.pitch;
    j["cmd_yaw"]              = cmd_yaw;
    j["cmd_pitch"]            = cmd_pitch;
    j["ekf_w"]                = ekf_w;
    j["ekf_angle"]            = ekf_angle;
    j["fire_count"]           = fire_count;
    j["fire"]                 = fire ? 1 : 0;

    j["yaw_err"]              = yaw_err;
    j["pitch_err"]            = pitch_err;
    j["aim_ready"]            = aim_ready ? 1 : 0;
    j["aim_ready_cnt"]        = aim_ready_cnt;

    // 诊断量（调参必备）
    j["t_process_ms"]         = t_process * 1000.0;
    j["t_flight"]             = t_flight;
    j["total_t"]              = t_process + SYS_DELAY_AFTER_NOW + t_flight;
    j["best_id"]              = tracked_id;
    j["err_at_hit"]           = err_at_hit;
    j["aligned"]              = aligned_to_armor ? 1 : 0;

    if (target) {
      auto x = target->ekf_x();
      j["r"]     = x[8];
      j["l"]     = x[9];
      j["h"]     = x[10];
      j["r_eff"] = (tracked_id == 1 || tracked_id == 3) ? (x[8] + x[9]) : x[8];
    }
    plotter.plot(j);
  }

  cv::destroyAllWindows();
  return 0;
}
