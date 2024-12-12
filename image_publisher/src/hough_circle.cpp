#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

bool process_hough()
{
    ROS_WARN("start process...");
    // 在需要时从ROS图像转换到OpenCV图像
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(latest_ros_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
        return false;
    }

    cv::Mat img = cv_ptr->image;//从ROS图像转为opencv
    det->undistort(img);//去畸变之后再识别
    int w = img.cols;
    int h = img.rows;
    Mat gray_img, bin_img;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    GaussianBlur(gray_img, gray_img, Size(9,9),2 ,2 );
    vector<Vec3f>circles;
    double mindist = 2;
    double min_r = 10;
    double max_r = 200;
    HoughCircles(gray_img, circles, HOUGH_GRADIENT, 1.5, mindist, 100, 100, min_r, max_r);
    for(size_t i = 0;i<circles.size();i++)
    {
        circle(img, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 255, 0), 3, 8);
        circle(img, Point(circles[i][0], circles[i][1]), 10, Scalar(255, 0, 0), -1, 8);
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    det_img_pub.publish(msg);
     Mat rotation_vector, t;
    Point2d l_u(circles[0][0]-circles[0][2], circles[0][1]-circles[0][2]);
    Point2d r_u(circles[0][0]+circles[0][2], circles[0][1]-circles[0][2]);
    Point2d l_d(circles[0][0]-circles[0][2], circles[0][1]+circles[0][2]);
    Point2d r_d(circles[0][0]+circles[0][2], circles[0][1]+circles[0][2]);
    std::vector<Point2d> image_points;
    image_points.push_back(l_u);//左上角
    image_points.push_back(r_u);
    image_points.push_back(l_d);
    image_points.push_back(r_d);
    // 3D 特征点世界坐标，与像素坐标对应，单位是cm
    std::vector<Point3d> model_points;
    model_points.push_back(Point3d(-29.9, -29.9, 0.0)); // 左上角 单位 cm
    model_points.push_back(Point3d(+29.9, -29.9, 0.0));
    model_points.push_back(Point3d(+29.9, +29.9, 0.0));
    model_points.push_back(Point3d(-29.9, +29.9, 0.0));
    
    // pnp求解
    //solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, t);

    // 设置响应
    if(circles.size()>0)
    {
        resp.is_detect = true;

        resp.min_type = -1;
        resp.x = t.at<double>(0,0);
        resp.y = t.at<double>(1,0);
        resp.z = t.at<double>(2,0);
    }

    return resp.is_detect;
}
