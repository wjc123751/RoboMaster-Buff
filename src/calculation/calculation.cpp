#include "calculation.h"

 /*pnp计算预测之后的yaw和pitch, 单位角度*/
void  calculation_angle(cv::Rect2d buff_Rect , double* yaw_angle , double* ptich_angle)
{
    /*相机内参矩阵*/
    cv::Mat camera_matrix = cv::Mat_<float>(3, 3);
    camera_matrix.at<float>(0, 0) = 1218.641027;
    camera_matrix.at<float>(0, 1) = 0.000000;
    camera_matrix.at<float>(0, 2) = 344.579207;
    camera_matrix.at<float>(1, 0) = 0.000000;
    camera_matrix.at<float>(1, 1) = 1214.391902;
    camera_matrix.at<float>(1, 2) = 307.645155;
    camera_matrix.at<float>(2, 0) = 0.000000;
    camera_matrix.at<float>(2, 1) = 0.000000;
    camera_matrix.at<float>(2, 2) = 1.000000;

    /*畸变系数*/
    cv::Mat distortion_coefficient = cv::Mat_<float>(5, 1);
    distortion_coefficient.at<float>(0) =0.004187;
    distortion_coefficient.at<float>(1) = -0.048726;
    distortion_coefficient.at<float>(2) =0.004211;
    distortion_coefficient.at<float>(3) = -0.009781;
    distortion_coefficient.at<float>(4) = 0.000000;

    /*实际坐标点*/
    std::vector<cv::Point3d> buffPoints;
    buffPoints.clear();
    buffPoints.push_back(cv::Point3d(-150,150,0));
    buffPoints.push_back(cv::Point3d(150,150,0));
    buffPoints.push_back(cv::Point3d(150,-150,0));
    buffPoints.push_back(cv::Point3d(-150,-150,0));

    /*像素坐标点*/
    cv::Point2d buff_uv1(buff_Rect.x, buff_Rect.y);
    cv::Point2d buff_uv2(buff_Rect.x + buff_Rect.width, buff_Rect.y);
    cv::Point2d buff_uv3(buff_Rect.x + buff_Rect.width, buff_Rect.y + buff_Rect.height);
    cv::Point2d buff_uv4(buff_Rect.x, buff_Rect.y + buff_Rect.height);

    std::vector<cv::Point2d> imgPoints;
    imgPoints.clear();
    imgPoints.push_back(buff_uv1);
    imgPoints.push_back(buff_uv2);
    imgPoints.push_back(buff_uv3);
    imgPoints.push_back(buff_uv4);

    // std::cout << "imgPoints:" << std::endl;
    // for (const auto& point : imgPoints) {
    //     std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    // }

    /*旋转向量和平移向量*/
    static cv::Mat rvec;
    static cv::Mat tvec;

     /*pnp解算外参矩阵*/
    //cv::solvePnPRansac(buffPoints, imgPoints, camera_matrix, distortion_coefficient, rvec, tvec, 0);
    cv::solvePnP(buffPoints, imgPoints, camera_matrix, distortion_coefficient, rvec, tvec);

    // cv::Mat rotM = cv::Mat::eye(3, 3, CV_64F);
    // cv::Mat rotT = cv::Mat::eye(3, 3, CV_64F);

    // Rodrigues(rvec, rotM);
    // // Rodrigues(tvec, rotT);

    // cv::Mat camera_position = -rotM.t() * tvec;

    // float distance = std::sqrt(std::pow(camera_position.at<double>(0) * 0.001, 2) + std::pow(camera_position.at<double>(1), 2) * 0.001
    //                                      + std::pow(camera_position.at<double>(2)  * 0.001, 2) );

    // std::cout << " atan(tvec.at<double>(0) / tvec.at<double>(2))" <<  atan(tvec.at<double>(0) / tvec.at<double>(2)) << std::endl;
    *yaw_angle = atan(tvec.at<double>(0) / tvec.at<double>(2)) * 180/  CV_PI;
    *ptich_angle = atan(tvec.at<double>(1) / tvec.at<double>(2)) * 180/  CV_PI;

    // std::cout << "yaw_angle: " << yaw_angle << std::endl;
    // std::cout << "ptich_angle: " << ptich_angle << std::endl;

}

 /*pnp计算车和大符之间的深度, 单位mm*/
float  calculation_distance(cv::Rect2d buff_Rect)
{
    /*相机内参矩阵*/
    cv::Mat camera_matrix = cv::Mat_<float>(3, 3);
    camera_matrix.at<float>(0, 0) = 1218.641027;
    camera_matrix.at<float>(0, 1) = 0.000000;
    camera_matrix.at<float>(0, 2) = 344.579207;
    camera_matrix.at<float>(1, 0) = 0.000000;
    camera_matrix.at<float>(1, 1) = 1214.391902;
    camera_matrix.at<float>(1, 2) = 307.645155;
    camera_matrix.at<float>(2, 0) = 0.000000;
    camera_matrix.at<float>(2, 1) = 0.000000;
    camera_matrix.at<float>(2, 2) = 1.000000;

    /*畸变系数*/
    cv::Mat distortion_coefficient = cv::Mat_<float>(5, 1);
    distortion_coefficient.at<float>(0) =0.004187;
    distortion_coefficient.at<float>(1) = -0.048726;
    distortion_coefficient.at<float>(2) =0.004211;
    distortion_coefficient.at<float>(3) = -0.009781;
    distortion_coefficient.at<float>(4) = 0.000000;

    /*实际坐标点*/
    std::vector<cv::Point3d> buffPoints;
    buffPoints.clear();
    buffPoints.push_back(cv::Point3d(-150,150,0));
    buffPoints.push_back(cv::Point3d(150,150,0));
    buffPoints.push_back(cv::Point3d(150,-150,0));
    buffPoints.push_back(cv::Point3d(-150,-150,0));

    /*像素坐标点*/
    cv::Point2d buff_uv1(buff_Rect.x, buff_Rect.y);
    cv::Point2d buff_uv2(buff_Rect.x + buff_Rect.width, buff_Rect.y);
    cv::Point2d buff_uv3(buff_Rect.x + buff_Rect.width, buff_Rect.y + buff_Rect.height);
    cv::Point2d buff_uv4(buff_Rect.x, buff_Rect.y + buff_Rect.height);

    std::vector<cv::Point2d> imgPoints;
    imgPoints.clear();
    imgPoints.push_back(buff_uv1);
    imgPoints.push_back(buff_uv2);
    imgPoints.push_back(buff_uv3);
    imgPoints.push_back(buff_uv4);

    // std::cout << "imgPoints:" << std::endl;
    // for (const auto& point : imgPoints) {
    //     std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    // }

    /*旋转向量和平移向量*/
    static cv::Mat rvec;
    static cv::Mat tvec;

     /*pnp解算外参矩阵*/
    cv::solvePnP(buffPoints, imgPoints, camera_matrix, distortion_coefficient, rvec, tvec);

    double dis =  tvec.at<double>(2);
    // std::cout << "dis = " << dis << std::endl;

    return dis;
}
