#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>

#include <thread> //添加线程支持

#include <X11/Xlib.h>

bool command = false;

std::string topic_img = "/camera/rgb/image_raw";

std::string save_path = ros::package::getPath("take_photo") + "/img/";

cv::Mat frame;

int count = 0;

bool command_Callback(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
    command =! command;
    ROS_INFO("camera already take photo [%s]",command==true?"YES":"NO");

    res.success = true;
    res.message = "camera already take photo!";
}

void show_photo(cv::Mat frame){
    cv::imshow("photo", frame);
    // cv::waitKey(3000);  //显示5秒
    // cv::destroyWindow("photo");
}

void callback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("视频流", frame);
        int key = cv::waitKey(1);
        if (key == 'q') {
            ROS_INFO("Exiting...");
            ros::shutdown();
        } else if (command) {
            std::string time_str = std::to_string(ros::Time::now().toSec());
            // std::string image_name = time_str + ".jpg";
            std::string image_name = "1.jpg";
            std::string full_path = save_path + image_name;
            cv::imwrite(full_path, frame);
            std::thread t(show_photo,frame); //在新线程中显示图像
            t.detach(); //分离线程
            command =! command;
            ROS_INFO("Saved image to %s", full_path.c_str());
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "take_photo_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic_img, 1, callback);
    ros::ServiceServer service = nh.advertiseService("/camera_command",command_Callback);    
    ros::spin();
    return 0;
}
