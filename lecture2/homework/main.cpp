#include <iostream>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "tasks/yolos/yolov5.hpp"
#include "tasks/armor.hpp"

int main()
{
    Camera cam;
    if (!cam.is_open_)
    {
        std::cerr << "相机打开失败！" << std::endl;
        return -1;
    }

    std::string yolo_config = "configs/yolo.yaml";
    bool debug_mode = false;
    auto_aim::YOLOV5 detector(yolo_config, debug_mode);

    cv::Mat frame;
    int frame_cnt = 0;
    while (true)
    {
        bool ok = cam.read(frame);
        if (!ok)
        {
            std::cerr << "图像读取失败" << std::endl;
            continue;
        }
        frame_cnt++;

        std::list<auto_aim::Armor> armor_list = detector.detect(frame, frame_cnt);

        for (const auto& armor : armor_list)
        {
            cv::rectangle(frame, armor.box, cv::Scalar(0, 255, 0), 2);

            for (const auto& pt : armor.points)
            {
                cv::circle(frame, pt, 4, cv::Scalar(0, 0, 255), -1);
            }

            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                cv::line(frame, armor.points[i], armor.points[j], cv::Scalar(0, 255, 0), 2);
            }
        
            std::string color_str = auto_aim::COLORS[armor.color];
            std::string name_str  = auto_aim::ARMOR_NAMES[armor.name];
            std::string info = color_str + " " + name_str;

            cv::putText(frame,
                        info,
                        cv::Point(armor.box.x, std::max(armor.box.y - 10, 20)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        3);
        }


        cv::imshow("YOLO Armor Detection", frame);
        int key = cv::waitKey(1);
        if (key == 'q') 
        {
            break;
        }
    }

    return 0;
}



