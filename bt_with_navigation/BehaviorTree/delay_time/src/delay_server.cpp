#include<ros/ros.h>
#include<delay_time/Delay.h>

bool Callback(delay_time::Delay::Request &req, delay_time::Delay::Response &res){
    ros::Duration delay_time(req.delay_time);
    delay_time.sleep();
    res.success = true;
}

int main(int argc,char** argv){
    ros::init(argc,argv,"delay_node");
    ros::NodeHandle nh;
    ros::ServiceServer dalay_srv = nh.advertiseService("/delay_time",Callback);
    ros::spin();
    return 0;
}
