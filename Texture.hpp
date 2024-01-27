//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        Eigen::Vector2i p1{(int)floor(u_img - 0.5), (int)floor(v_img - 0.5)};
        Eigen::Vector2i p2{(int)floor(u_img + 0.5), (int)floor(v_img - 0.5)};
        Eigen::Vector2i p3{(int)floor(u_img - 0.5), (int)floor(v_img + 0.5)};
        Eigen::Vector2i p4{(int)floor(u_img + 0.5), (int)floor(v_img + 0.5)};
        auto s = u_img - ((double)p1.x() + 0.5);
        auto t = v_img - ((double)p1.y() + 0.5);
        auto color1 = image_data.at<cv::Vec3b>(p1.y(), p1.x()) * (1 - s) + image_data.at<cv::Vec3b>(p2.y(), p2.x()) * s;
        auto color2 = image_data.at<cv::Vec3b>(p3.y(), p3.x()) * (1 - s) + image_data.at<cv::Vec3b>(p4.y(), p4.x()) * s;
        auto color = color1 * (1 - t) + color2 * t;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
