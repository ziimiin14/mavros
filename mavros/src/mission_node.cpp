/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>





mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      //      ("mavros/setpoint_position/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //geometry_msgs::PoseStamped pose;
    //pose.pose.position.x = 0;
    //pose.pose.position.y = 0;
    //pose.pose.position.z = 2;

    

    //send a few setpoints before starting
    //for(int i = 100; ros::ok() && i > 0; --i){
      //  local_pos_pub.publish(pose);
        //ros::spinOnce();
        //rate.sleep();
   // }

  

    mavros_msgs::SetMode mission_set_mode;
    mission_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::WaypointPush wp_push_srv;
    mavros_msgs::Waypoint wp;

     // WP 0
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3978206;
    wp.y_long         = 8.543987;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
    // WP 1
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3962527;
    wp.y_long         = 8.5467917;
    wp.z_alt          = 20;
	wp.param1			= 10;
	wp.param3			= 2;
	wp.param4			= 1;
    wp_push_srv.request.waypoints.push_back(wp);
    
    // WP 2
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3977783;
    wp.y_long         = 8.547906;
    wp.z_alt          = 20;
    wp_push_srv.request.waypoints.push_back(wp);

    // WP 3
    wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();



    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        if (current_state.mode != "AUTO.MISSION") {
            if( set_mode_client.call(mission_set_mode) &&
                mission_set_mode.response.mode_sent){
                ROS_INFO("AUTO.MISSION enabled");
            }
        }
    }
    else{
        ROS_ERROR("Send waypoints FAILED.");
    }

    if(arming_client.call(arm_cmd)&& arm_cmd.response.success){
        ROS_INFO("VEHICLE ARMED");
    }

    while(ros::ok()){
        /*if( current_state.mode != "AUTO.MISSION" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(mission_set_mode) &&
                mission_set_mode.response.mode_sent){
                ROS_INFO("AUTO MISSION enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        

        local_pos_pub.publish(pose);*/

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
