#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "tasks/charge_types.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char** argv)
{
    std::string apriltag_config = "configs/apriltag.yaml";
    auto_charge::AprilTagDetector detector(apriltag_config);

    Camera cam;
    cv::Mat frame;

    tools::logger()->info("===== AprilTag附加作业程序启动 =====");
    tools::logger()->info("目标Tag ID: 10, 18, 24 | 按ESC退出窗口");

    while (true)
    {
        if (!cam.read(frame))
        {
            tools::logger()->warn("相机读取帧失败");
            continue;
        }

        std::vector<auto_charge::TagDetection> tags = detector.detect(frame);

        for (const auto& tag : tags)
        {
            std::vector<cv::Point> poly;
            for (const auto& pt : tag.corners)
            {
                poly.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
            }
            cv::polylines(frame, poly, true, cv::Scalar(0, 255, 0), 2);

            tools::draw_point(frame, tag.center, cv::Scalar(0, 0, 255), 4);

            std::string id_text = "ID:" + std::to_string(tag.id);
            tools::draw_text(frame, id_text, cv::Point(tag.center.x + 10, tag.center.y),
                             cv::Scalar(255, 255, 0), 0.8, 2);
        }

        cv::imshow("AprilTag Detection", frame);
        int key = cv::waitKey(1);
        if (key == 27) 
        {
            tools::logger()->info("程序退出");
            break;
        }
    }
    cv::destroyAllWindows();
    return 0;
}


