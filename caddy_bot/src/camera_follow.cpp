#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>

#include <iostream>
#include <iomanip>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <cmath>

#include <actionlib_msgs/GoalID.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher pub_goal;
ros::Publisher pub_goal_cancel;

float GetDegree (geometry_msgs::PointStamped map_point1, geometry_msgs::PointStamped map_point2)
{
    float delta_x = map_point1.point.x - map_point2.point.x;
    float delta_y = map_point1.point.y - map_point2.point.y;

    float diff_deg = atan2f(delta_y, delta_x);

    // float diff_deg = 360 - (180 - (atan2f(delta_y, delta_x)* 180 / 3.14));

    return diff_deg;
}


void buildGoal(geometry_msgs::PointStamped map_point, bool flag)
{

    geometry_msgs::PoseStamped rpyGoal;

    rpyGoal.header.frame_id = "odom";
    rpyGoal.header.stamp = ros::Time::now();

    if(flag){
        rpyGoal.pose.position.x = 0; //specify x goal
        rpyGoal.pose.position.y = 0; //specify y goal
        rpyGoal.pose.position.z = 0;
    }else{
        // Specify x and y goal
        rpyGoal.pose.position.x = map_point.point.x; //specify x goal
        rpyGoal.pose.position.y = map_point.point.y; //specify y goal
        rpyGoal.pose.position.z = 0;
    }

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    tf::Matrix3x3 rot_euler;
    tf::Quaternion rot_quat;

    // Calculate quaternion
    float x_curr = 0, y_curr = 0; // set current coords.
    float x_next = map_point.point.x, y_next = map_point.point.y; // set coords. of next waypoint
    float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
    float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
    yaw_curr = atan2(delta_y, delta_x);

    // Specify quaternions
    // rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    rot_euler.getRotation(rot_quat);

    rpyGoal.pose.orientation.x = rot_quat.getX();
    rpyGoal.pose.orientation.y = rot_quat.getY();
    rpyGoal.pose.orientation.z = rot_quat.getZ();
    rpyGoal.pose.orientation.w = rot_quat.getW();

    pub_goal.publish(rpyGoal);
}

void transformPoint(const tf::TransformListener& listener)
{
        
    actionlib_msgs::GoalID goal_cancel;
    geometry_msgs::PointStamped base_point0;
    base_point0.header.frame_id = "base_point0";
    base_point0.header.stamp = ros::Time();
    base_point0.point.x = 0.0;
    base_point0.point.y = 0.0;
    base_point0.point.z = 0.0;
    geometry_msgs::PointStamped base_point1, base_point2;
    geometry_msgs::PointStamped person0_point;
    person0_point.header.frame_id = "person0";
    person0_point.header.stamp = ros::Time();
    person0_point.point.x = 0.0;
    person0_point.point.y = 0.0;
    person0_point.point.z = 0.0;


    try{
        listener.transformPoint("odom", person0_point, base_point1);
        listener.transformPoint("camera_link", person0_point, base_point2);

        // ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> person0: (%.2f, %.2f, %.2f) at time %.2f",
        //     person0_point.point.x, person0_point.point.y, person0_point.point.z,
        //     base_point2.point.x, base_point2.point.y, base_point2.point.z, base_point2.header.stamp.toSec());
        // ROS_INFO("time (%.2f, %.2f)",ros::Time::now().toSec(),base_point2.header.stamp.toSec());

        if (ros::Time::now().toSec() - base_point2.header.stamp.toSec() > 0.1){
            pub_goal_cancel.publish(goal_cancel);
        }else{
            float yaw_gap = GetDegree(person0_point, base_point2);
            bool yaw_gap_flag = false;

            if (yaw_gap * 180 / 3.14 > 0 ){
                // ROS_INFO("yaw_gap: %.2f",180 - (yaw_gap * 180 / 3.14));
                if (180 - (yaw_gap * 180 / 3.14) > 10){
                    yaw_gap_flag = true;
                }
            }else{
                // ROS_INFO("yaw_gap: %.2f",(yaw_gap * 180 / 3.14)+180);
                if ((yaw_gap * 180 / 3.14) + 180 > 10){
                    yaw_gap_flag = true;
                }
            }
            buildGoal(base_point1,false);
        }
    }
    catch(tf::TransformException& ex){
        // ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"odom\": %s", ex.what());
        // ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"camera_link\": %s", ex.what());
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_follow");
    ros::NodeHandle n;

    pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    pub_goal_cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);

    tf::TransformListener listener(ros::Duration(6));

    //we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(0.16), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();

}