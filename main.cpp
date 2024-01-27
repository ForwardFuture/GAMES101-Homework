#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[0] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    auto &p_0 = control_points[0];
    auto &p_1 = control_points[1];
    auto &p_2 = control_points[2];
    auto &p_3 = control_points[3];

    auto a_1 = p_0 * (1 - t) + p_1 * t;
    auto a_2 = p_1 * (1 - t) + p_2 * t;
    auto a_3 = p_2 * (1 - t) + p_3 * t;
    
    auto b_1 = a_1 * (1 - t) + a_2 * t;
    auto b_2 = a_2 * (1 - t) + a_3 * t;

    auto c = b_1 * (1 - t) + b_2 * t;

    return cv::Point2f(c.x, c.y);

}

void antialiasingShading(const cv::Point2f &lastpoint, const cv::Point2f &point, cv::Mat &window)
{
    float d = 2, extendfactor = 0;

    cv::Vec2f tanVec = cv::Vec2f{point.x - lastpoint.x, point.y - lastpoint.y};
    tanVec = tanVec / sqrt(tanVec[0] * tanVec[0] + tanVec[1] * tanVec[1]);

    cv::Vec2f normalVec = cv::Vec2f{-tanVec[1], tanVec[0]} * d;

    cv::Point2f q1 = cv::Point2f{lastpoint.x + normalVec[0] - tanVec[0] * extendfactor,
                                lastpoint.y + normalVec[1] - tanVec[1] * extendfactor};
    cv::Point2f q2 = cv::Point2f{lastpoint.x - normalVec[0] - tanVec[0] * extendfactor,
                                lastpoint.y - normalVec[1] - tanVec[1] * extendfactor};
    cv::Point2f q3 = cv::Point2f{point.x + normalVec[0] + tanVec[0] * extendfactor,
                                point.y + normalVec[1] + tanVec[1] * extendfactor};
    cv::Point2f q4 = cv::Point2f{point.x - normalVec[0] + tanVec[0] * extendfactor,
                                point.y - normalVec[1] + tanVec[1] * extendfactor};

    int minx = std::min(q1.x, std::min(q2.x, std::min(q3.x, q4.x)));
    int maxx = std::max(q1.x, std::max(q2.x, std::max(q3.x, q4.x)));
    int miny = std::min(q1.y, std::min(q2.y, std::min(q3.y, q4.y)));
    int maxy = std::max(q1.y, std::max(q2.y, std::max(q3.y, q4.y)));

    for(int i = minx; i <= maxx; i++)
        for(int j = miny; j <= maxy; j++){

            cv::Vec2f newVec = cv::Vec2f{(float)i + 0.5 - lastpoint.x, (float)j + 0.5 - lastpoint.y};
            float len = sqrt(newVec[0] * newVec[0] + newVec[1] * newVec[1]);
            newVec = newVec / len;
            
            float dis = std::min((float)sqrt(1 - pow(tanVec.dot(newVec), 2)) * len, d);

            cv::Point2f p1 = cv::Point2f{(float)i + 0.25, (float)j + 0.25};
            cv::Point2f p2 = cv::Point2f{(float)i + 0.75, (float)j + 0.25};
            cv::Point2f p3 = cv::Point2f{(float)i + 0.25, (float)j + 0.75};
            cv::Point2f p4 = cv::Point2f{(float)i + 0.75, (float)j + 0.75};

            int param = 0;

            newVec = cv::Vec2f{p1.x - lastpoint.x, p1.y - lastpoint.y};
            len = sqrt(newVec[0] * newVec[0] + newVec[1] * newVec[1]);
            newVec = newVec / len;
            param += sqrt(1 - pow(tanVec.dot(newVec), 2)) * len <= d;

            newVec = cv::Vec2f{p2.x - lastpoint.x, p2.y - lastpoint.y};
            len = sqrt(newVec[0] * newVec[0] + newVec[1] * newVec[1]);
            newVec = newVec / len;
            param += sqrt(1 - pow(tanVec.dot(newVec), 2)) * len <= d;
            
            newVec = cv::Vec2f{p3.x - lastpoint.x, p3.y - lastpoint.y};
            len = sqrt(newVec[0] * newVec[0] + newVec[1] * newVec[1]);
            newVec = newVec / len;
            param += sqrt(1 - pow(tanVec.dot(newVec), 2)) * len <= d;

            newVec = cv::Vec2f{p4.x - lastpoint.x, p4.y - lastpoint.y};
            len = sqrt(newVec[0] * newVec[0] + newVec[1] * newVec[1]);
            newVec = newVec / len;
            param += sqrt(1 - pow(tanVec.dot(newVec), 2)) * len <= d;

            window.at<cv::Vec3b>(j, i)[1] = std::max(window.at<cv::Vec3b>(j, i)[1], (uchar)(255.0 * (1 - dis / d) * (float)param / 4.0));
        }
}


void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    cv::Point2f lastpoint = recursive_bezier(control_points, 0);

    for (double t = 0.001; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        antialiasingShading(lastpoint, point, window);
        lastpoint = point;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);
            cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);
            
            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
