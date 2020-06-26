// General headers required to run ROS script
#include <ros/ros.h>
#include <iostream>
// Headers needed for data obtaining
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <geometry_msgs/PoseStamped.h>

//For some math calculation
#include <cmath>

//For quarternion
#include <mavros/frame_tf.h>
using namespace mavros;

int loopRate = 0.05;
//float focal_length = 83.36;
float offset_lidar = 0.06;


geometry_msgs::PoseStamped msg_vicon_pose;
sensor_msgs::Range current_range;
mavros_msgs::OpticalFlowRad current_of;
sensor_msgs::Imu current_imu;




void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    ROS_INFO("Current Rotation rate(%s):{roll_rate=[%f], pitch_rate=[%f], yaw_rate=[%f]}", imu_msg->header.frame_id.c_str(), imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
}


void ds_cb(const sensor_msgs::Range::ConstPtr& range_msg)
{
    current_range.range = range_msg->range + offset_lidar;
    ROS_INFO("Current Range(%s): [%f]",range_msg->header.frame_id.c_str(),current_range.range);
}

void of_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& of_msg)
{
    ROS_INFO("Optical Flow(%s):{z_integral(pixel)=[%f], y_integral(pixel)=[%f]}",of_msg->header.frame_id.c_str(), of_msg->integrated_x, of_msg->integrated_y);
}

void Callback_fcu_pose(const geometry_msgs::PoseStamped msg_fcu_pose_dummy)
{
    ROS_INFO("Got data : %f, %f, %f", msg_fcu_pose_dummy.pose.position.x, msg_fcu_pose_dummy.pose.position.y, msg_fcu_pose_dummy.pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_sub");
    ros::NodeHandle n;

    //ros::Publisher vicon_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",loopRate);
    //Sub lidar data
    ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::Range>("/distance_sensor/lidar0",loopRate,ds_cb);
    ros::Subscriber lidar_sub1 = n.subscribe<sensor_msgs::Range>("/distance_sensor/lidar1",loopRate,ds_cb);
    ros::Subscriber lidar_sub2 = n.subscribe<sensor_msgs::Range>("/distance_sensor/lidar2",loopRate,ds_cb);
    ros::Subscriber lidar_sub3 = n.subscribe<sensor_msgs::Range>("/distance_sensor/lidar3",loopRate,ds_cb);

    //Sub optical flow data
    ros::Subscriber of_sub = n.subscribe<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad",loopRate,of_cb);

    //Sub imu data
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data",loopRate,imu_cb);

    auto q = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
	
/*    ros::Subscriber vicon_sub = n.subscribe("/mavros/local_position/pose",loopRate, Callback_fcu_pose);

    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        loop_rate.sleep();
        msg_vicon_pose.header.stamp = ros::Time::now();
        //msg_vicon_pose.header.frame_id = "fcu"; //optional. Works fine without frame_id
        msg_vicon_pose.pose.position.x = 2;
        msg_vicon_pose.pose.position.y = 3;
        msg_vicon_pose.pose.position.z = 5;

        vicon_pub.publish(msg_vicon_pose);

        ros::spinOnce();
    }*/
    ros::spin();

    return 0;
}

