#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

class UwbFollow
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_floatter1_;
    ros::Subscriber sub_floatter2_;
    std_msgs::Float32 flaotter1;
    std_msgs::Float32 flaotter2;
    tf::TransformBroadcaster tf_publisher_;
public:
    UwbFollow()
    {
        sub_floatter1_ = nh_.subscribe("/floatter1", 256, &UwbFollow::floatter1Callback, this);
        sub_floatter2_ = nh_.subscribe("/floatter2", 256, &UwbFollow::floatter2Callback, this);
    }
    void floatter1Callback(const std_msgs::Float32 &message)
    {
        flaotter1 = message;
    }
    void floatter2Callback(const std_msgs::Float32 &message)
    {
        flaotter2 = message;
        double d = 0.42;
        double x;
        double y;
        if ((flaotter1.data != 0.0) && (flaotter2.data != 0.0))
        {
            try
            {
                x = (pow(flaotter1.data, 2) - pow(d, 2) - pow(flaotter2.data, 2)) / (-2 * d);
                y = sqrt(pow(flaotter1.data, 2) - pow(x, 2));
                ros::Time current_time = ros::Time::now();
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "uwb_link";
                odom_trans.child_frame_id = "person";
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                // odom_trans.transform.translation.z = ;
                odom_trans.transform.rotation = odom_quat;
                tf_publisher_.sendTransform(odom_trans);
            }
            catch(int expn)
            {
                
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker");
    UwbFollow a;
    
    ros::spin();
}