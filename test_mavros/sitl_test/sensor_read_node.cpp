#include <ros/ros.h>
#include <sensor_msgs/Range.h>
//#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Header.h>
#include <mavros_msgs/OpticalFlowRad.h>

//sensor_msgs::Range current_range;
void ds_cb(const sensor_msgs::Range::ConstPtr& msg){
   
    ROS_INFO("Current Range(%s): [%f]",msg->header.frame_id.c_str(),msg->range);
}

void of_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &msg){
	
    ROS_INFO("Optical Flow(%s):{z_integral(rad)=[%f], y_integral(rad)=[%f]}",msg->header.frame_id.c_str(), msg->integrated_x, msg->integrated_y);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"sensor_read_node");
    ros::NodeHandle nh;

    ros::Subscriber ds_sub =nh.subscribe<sensor_msgs::Range>("/distance_sensor/lidar0",10,ds_cb);
    ros::Subscriber ds_sub1 =nh.subscribe<sensor_msgs::Range>("/distance_sensor/lidar1",10,ds_cb);
    ros::Subscriber ds_sub2 =nh.subscribe<sensor_msgs::Range>("/distance_sensor/lidar2",10,ds_cb);
    ros::Subscriber ds_sub3 =nh.subscribe<sensor_msgs::Range>("/distance_sensor/lidar3",10,ds_cb);
    ros::Subscriber of_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad",10,of_cb);
    
    //ROS_INFO("Vehicle armed");
    ros::spin();

    return 0;

}
	
