#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "start_stream_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::string gst_str = "v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080 ! videoconvert ! appsink";
    
    cv::VideoCapture cap(gst_str, cv::CAP_GSTREAMER);
    Mat image;
    
    sensor_msgs::ImagePtr msg;

    if(!cap.isOpened())
    {
        ROS_ERROR("Find No Device...");
        return -1;
    }

    ros::Rate loop_rate(5);  // Control the publish rate
    while (nh.ok()) 
    {
        cap.read(image);
        if (!image.empty()) 
        {
            flip(image, image, -1);
            // resize(image, image, Size(640,640));
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub.publish(msg);
            // imshow("111", image);
            // waitKey(1);
        }
        else
        {
            ROS_WARN("No Image...");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
