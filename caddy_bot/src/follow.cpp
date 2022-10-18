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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
// std::string utm_zone;
double utm_zone;
std::string path_local, path_abs;


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("caddy_bot") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("caddy_bot") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < (numWaypoints + 1); i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    // RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);
    double lat = lati_input;
    lat = lat + 0.00001;
    double lon = longi_input;
    double east, north;
    int zone;
    if(lon < 0){
        zone = (lon + 180) / 6 + 1;
    }
    else{
        zone = lon / 6 + 31;
    }

    double lat_rad = lat * M_PI/180;
    double lon_rad = lon * M_PI/180;

    double phi = lat_rad;
    double lambda = lon_rad;
    double lambda0 = (zone * 6 -183) * M_PI/180;
    double sm_a = 6378137;
    double sm_b = 6356752.31;

    double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    double nu2 = ep2*pow(cos(phi), 2.0);
    double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    double l = lambda - lambda0;
    double t = tan(phi);
    double t2 = t * t;

    double l3coef = 1 - t2 + nu2;
    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    east = N * cos(phi) * l + 
        (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + 
        (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + 
        (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    double n = (sm_a - sm_b) / (sm_a + sm_b);
    double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
    double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));

    north = ArcLengthMeridian + 
            (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + 
            (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + 
            (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + 
            (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));

    double easting = east * 0.9996 + 500000;
    double northing = north * 0.9996;

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = easting;
    UTM_point_output.point.y = northing;
    UTM_point_output.point.z = 0;
    
    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "camera_link", time_now, ros::Duration(0.5));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

// geometry_msgs::PointStamped TFtoMapPoint()
// {
//       //we'll create a point in the person0 frame that we'd like to transform to the camera_link frame
//     geometry_msgs::PointStamped base_point;
//     geometry_msgs::PointStamped person_point;
//     bool notDone = true;
//     tf::TransformListener listener;
//     person_point.header.frame_id = "person0";

//   //we'll just use the most recent transform available for our simple example
//     person_point.header.stamp = ros::Time();

//   //just an arbitrary point in space
// //   person_point.point.x = 1.0;
// //   person_point.point.y = 0.2;
// //   person_point.point.z = 0.0;

//     // person_point.point.x = 0.0;
//     // person_point.point.y = 0.0;
//     // person_point.point.z = 0.0;
//     while(notDone)
//     {
//         try
//         {
//             listener.transformPoint("camera_link", person_point, base_point);

//             ROS_INFO("person11: (%.2f, %.2f. %.2f) -----> camera_link: (%.2f, %.2f, %.2f) at time %.2f",
//                 person_point.point.x, person_point.point.y, person_point.point.z,
//                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//             notDone = false;
//         }
//         catch(tf::TransformException& ex)
//         {
//             ROS_ERROR("Received an exception trying to transform a point from \"person11\" to \"camera_link\": %s", ex.what());
//         }
//     }
//     return base_point;


// try{
//     geometry_msgs::PointStamped base_point;
//     listener.waitForTransform("odom", "person0", time_now, ros::Duration(0.5));
//     listener.transformPoint("odom", person_point, base_point);

//     ROS_INFO("person0: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
//         person_point.point.x, person_point.point.y, person_point.point.z,
//         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//   }
//   catch(tf::TransformException& ex){
//     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"odom\": %s", ex.what());
//   }

    // geometry_msgs::PointStamped map_point_output;
    // bool notDone = true;
    // tf::TransformListener listener; //create transformlistener object called listener
    // ros::Time time_now = ros::Time::now();
    // while(notDone)
    // {
    //     try
    //     {
    //         UTM_point.header.stamp = ros::Time::now();
    //         listener.waitForTransform("odom", "person0", time_now, ros::Duration(0.5));
    //         listener.transformPoint("odom", "person0", map_point_output);
    //         notDone = false;
    //     }
    //     catch (tf::TransformException& ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         ros::Duration(0.01).sleep();
    //         //return;
    //     }
    // }
    // return map_point_output;
// }

// move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point1, geometry_msgs::PointStamped map_point2)
// {
//     move_base_msgs::MoveBaseGoal goal;
//     tf::Matrix3x3 rot_euler;
//     tf::Quaternion rot_quat;
//     float yaw_curr = map_point1.point.z - map_point2.point.z, pitch_curr = 0, roll_curr = 0;

//     rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
//     rot_euler.getRotation(rot_quat);

//     // float x = map_point2.point.x*cos(yaw_curr) - map_point2.point.y*sin(yaw_curr);
//     // float y = map_point2.point.x*sin(yaw_curr) + map_point2.point.y*cos(yaw_curr);

//     goal.target_pose.header.frame_id = "odom";
//     goal.target_pose.header.frame_id = "odom";
//     goal.target_pose.header.stamp = ros::Time::now();
//     goal.target_pose.pose.position.x = map_point2.point.x;
//     goal.target_pose.pose.position.y = map_point2.point.y;
//     // goal.target_pose.pose.position.z = map_point2.point.z;
//     // goal.target_pose.pose.position.x = x;
//     // goal.target_pose.pose.position.y = y;
//     goal.target_pose.pose.orientation.x = rot_quat.getX();
//     goal.target_pose.pose.orientation.y = rot_quat.getY();
//     goal.target_pose.pose.orientation.z = rot_quat.getZ();
//     goal.target_pose.pose.orientation.w = rot_quat.getW();
    
//     return goal;
// }



move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    tf::Matrix3x3 rot_euler;
    tf::Quaternion rot_quat;

    // Calculate quaternion
    float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
    float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
    float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
    float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
    yaw_curr = atan2(delta_y, delta_x);

    // Specify quaternions
    rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    rot_euler.getRotation(rot_quat);

    goal.target_pose.pose.orientation.x = rot_quat.getX();
    goal.target_pose.pose.orientation.y = rot_quat.getY();
    goal.target_pose.pose.orientation.z = rot_quat.getZ();
    goal.target_pose.pose.orientation.w = rot_quat.getW();

    return goal;
}


// move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point)
// {
//     move_base_msgs::MoveBaseGoal goal;
//     tf::Matrix3x3 rot_euler;
//     tf::Quaternion rot_quat;

//     // goal.target_pose.header.frame_id = "odom";
//     goal.target_pose.header.frame_id = "odom";
//     goal.target_pose.header.stamp = ros::Time::now();

//     float x = 0.0;
//     float y = 0.0;

//     float d = map_point1.point.z - map_point2.point.z -1.5708;
//     float dX = x - map_point2.point.x;
//     float dY = y - map_point2.point.y;

//     float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
//     yaw_curr = atan2(map_point2.point.y, map_point2.point.x);

//     float rad = abs(d);

//     float cosD = cos(rad);
//     float sinD = sin(rad);

//     if(d >= 0){
//         x = dX * cosD + dY * sinD;
//         y = dY * cosD - dX * sinD;
//     } else {
//         x = dX * cosD - dY * sinD;
//         y = dY * cosD + dX * sinD;
//     }

//     // x += map_point.point.x;
//     // y += map_point.point.y;
    
//     rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
//     rot_euler.getRotation(rot_quat);

//     goal.target_pose.pose.position.x = x;
//     goal.target_pose.pose.position.y = y;
//     goal.target_pose.pose.orientation.z = rot_quat.getZ();
//     goal.target_pose.pose.orientation.w = rot_quat.getW();

//     return goal;
// }

void transformPoint(const tf::TransformListener& listener){
    
    geometry_msgs::PointStamped base_point0;
    base_point0.header.frame_id = "base_point0";
    base_point0.header.stamp = ros::Time();
    base_point0.point.x = 0.0;
    base_point0.point.y = 0.0;
    base_point0.point.z = 0.0;
    geometry_msgs::PointStamped base_point1;
    geometry_msgs::PointStamped person0_point;
    person0_point.header.frame_id = "person0";
    person0_point.header.stamp = ros::Time();
    person0_point.point.x = 0.0;
    person0_point.point.y = 0.0;
    person0_point.point.z = 0.0;

    try{
        listener.transformPoint("odom", person0_point, base_point1);

        ROS_INFO("odom: (%.2f, %.2f. %.2f) -----> base_footprint: (%.2f, %.2f, %.2f) at time %.2f",
            person0_point.point.x, person0_point.point.y, person0_point.point.z,
            base_point1.point.x, base_point1.point.y, base_point1.point.z, base_point1.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_footprint\" to \"odom\": %s", ex.what());
    }

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal = buildGoal(base_point1,base_point1);
    ROS_INFO("goal: (%.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ac.sendGoal(goal);

    // map_point = TFtoMapPoint();
    // geometry_msgs::PointStamped odom_point;
    // geometry_msgs::PointStamped camera_point;
    // geometry_msgs::PointStamped base_point1;
    // geometry_msgs::PointStamped base_point2;

    // odom_point.header.frame_id = "odom";
    // odom_point.header.stamp = ros::Time();
    // odom_point.point.x = 0.0;
    // odom_point.point.y = 0.0;
    // odom_point.point.z = 0.0;

    // camera_point.header.frame_id = "camera_link";
    // camera_point.header.stamp = ros::Time();
    // camera_point.point.x = 0.0;
    // camera_point.point.y = 0.0;
    // camera_point.point.z = 0.0;

    // try{
    //     listener.transformPoint("camera_link", odom_point, base_point1);

    //     ROS_INFO("odom: (%.2f, %.2f. %.2f) -----> base_footprint: (%.2f, %.2f, %.2f) at time %.2f",
    //         odom_point.point.x, odom_point.point.y, odom_point.point.z,
    //         base_point1.point.x, base_point1.point.y, base_point1.point.z, base_point1.header.stamp.toSec());
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point from \"base_footprint\" to \"odom\": %s", ex.what());
    // }

    // try{
    //     listener.transformPoint("person0", camera_point, base_point2);

    //     ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> person0: (%.2f, %.2f, %.2f) at time %.2f",
    //         camera_point.point.x, camera_point.point.y, camera_point.point.z,
    //         base_point2.point.x, base_point2.point.y, base_point2.point.z, base_point2.header.stamp.toSec());
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"camera_link\": %s", ex.what());
    // }

    // MoveBaseClient ac("move_base", true);
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }
    // move_base_msgs::MoveBaseGoal goal;
    // goal = buildGoal(base_point1, base_point2);
    // ROS_INFO("goal: (%.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    // ac.sendGoal(goal);


    // ac.waitForResult();

    // geometry_msgs::PointStamped camera_point;
    // geometry_msgs::PointStamped person_point;
    // geometry_msgs::PointStamped base_point;

    // camera_point.header.frame_id = "odom";
    // camera_point.header.stamp = ros::Time();
    // camera_point.point.x = 0.0;
    // camera_point.point.y = 0.0;
    // camera_point.point.z = 0.0;

    // try{
    //     listener.transformPoint("person0", camera_point, base_point);

    //     ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> person0: (%.2f, %.2f, %.2f) at time %.2f",
    //         camera_point.point.x, camera_point.point.y, camera_point.point.z,
    //         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"camera_link\": %s", ex.what());
    // }

    // MoveBaseClient ac("move_base", true);
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }
    // move_base_msgs::MoveBaseGoal goal;
    // goal = buildGoal(base_point);
    // ROS_INFO("goal: (%.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

    // ac.sendGoal(goal);
    // ac.waitForResult();
    
    // person_point.header.frame_id = "person0";
    // person_point.header.stamp = ros::Time();
    // try{
    //     geometry_msgs::PointStamped base_point;
    //     listener.transformPoint("odom", person_point, base_point);

    //     ROS_INFO("person0: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
    //         person_point.point.x, person_point.point.y, person_point.point.z,
    //         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    //     move_base_msgs::MoveBaseGoal goal = buildGoal(base_point);
    //     ac.sendGoal(goal);
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"odom\": %s", ex.what());
    // }
    

  //we'll create a point in the person0 frame that we'd like to transform to the camera_link frame
//   geometry_msgs::PointStamped person_point;
//   person_point.header.frame_id = "person0";

  //we'll just use the most recent transform available for our simple example
//   person_point.header.stamp = ros::Time();

  //just an arbitrary point in space
//   person_point.point.x = 1.0;
//   person_point.point.y = 0.2;
//   person_point.point.z = 0.0;

//   person_point.point.x = 0.0;
//   person_point.point.y = 0.0;
//   person_point.point.z = 0.0;

//   try{
//     geometry_msgs::PointStamped base_point;
//     listener.transformPoint("camera_link", person_point, base_point);

//     ROS_INFO("person0: (%.2f, %.2f. %.2f) -----> camera_link: (%.2f, %.2f, %.2f) at time %.2f",
//         person_point.point.x, person_point.point.y, person_point.point.z,
//         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//   }
//   catch(tf::TransformException& ex){
//     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"camera_link\": %s", ex.what());
//   }

// try{
//     geometry_msgs::PointStamped base_point;
//     listener.waitForTransform("odom", "person0", time_now, ros::Duration(0.5));
//     listener.transformPoint("odom", person_point, base_point);

//     ROS_INFO("person0: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
//         person_point.point.x, person_point.point.y, person_point.point.z,
//         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//   }
//   catch(tf::TransformException& ex){
//     ROS_ERROR("Received an exception trying to transform a point from \"person0\" to \"odom\": %s", ex.what());
//   }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_follow");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
