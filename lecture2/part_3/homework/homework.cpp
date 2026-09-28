#include <iostream>
#include <opencv2/opencv.hpp>
int main()
{
    cv::Mat img=cv::imread("../assets/demo.jpg");
    if(img.empty())
    {
        std::cout<<"读取图片失败！检查路径"<<std::endl;
        return -1;
    }
    cv::Mat gray_img;
    cv::cvtColor(img,gray_img,cv::COLOR_BGR2GRAY);
    cv::imwrite("gray.jpg",gray_img);
    std::cout<<"灰度图已保存 gray.jpg"<<std::endl;
    cv::circle(gray_img,cv::Point(400,300),60,cv::Scalar(255),3);
    cv::imshow("homework gray",gray_img);
    std::cout<<"按任意键关闭窗口..."<<std::endl;
    cv::waitKey(0);
    return 0;
}
