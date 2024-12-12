#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void extractFramesFromBag(const std::string& bagFile, const std::string& topicName, const std::string& outputPath, int frameInterval) {
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topicName);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frameCount = 0;

    for (rosbag::MessageInstance const m : view) {
        sensor_msgs::Image::ConstPtr imageMsg = m.instantiate<sensor_msgs::Image>();
        if (imageMsg != nullptr) {
            if (frameCount % frameInterval == 0) {
                cv_bridge::CvImagePtr cvImagePtr;
                try {
                    cvImagePtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }

                std::string outputFile = outputPath + "/frame_" + std::to_string(frameCount) + ".png";
                cv::imwrite(outputFile, cvImagePtr->image);
                std::cout << "Saved frame " << frameCount << " to " << outputFile << std::endl;
            }
            frameCount++;
        }
    }

    bag.close();
}

int main(int argc, char** argv) 
{


    std::string bagFile = "/home/amov/tk1.bag";
    std::string topicName = "/camera/image";
    std::string outputPath = "/home/amov/trainimg/tk";
    int frameInterval = 2;

    extractFramesFromBag(bagFile, topicName, outputPath, frameInterval);

    return 0;
}
