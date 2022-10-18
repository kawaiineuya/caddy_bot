#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "tf/transform_broadcaster.h"

#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"
// #include "librealsense2/hpp/rs_processing.hpp"
// #include "librealsense2/rs_advanced_mode.hpp"

class ObjectDrawer
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_box_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    tf::TransformBroadcaster tf_publisher_;
    image_geometry::PinholeCameraModel cam_model_;
    darknet_ros_msgs::BoundingBoxes boxes;
    rs2_intrinsics intrinsic_;
public:
    ObjectDrawer()
        : it_(nh_)
    {
        std::string image_topic = nh_.resolveName("camera/aligned_depth_to_color/image_raw");
        sub_ = it_.subscribeCamera(image_topic, 1024, &ObjectDrawer::imageCb, this);
        sub_box_ = nh_.subscribe("darknet_ros/bounding_boxes", 256, &ObjectDrawer::boundingCallback, this);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        try
        {
          input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
          image = input_bridge->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }
        intrinsic_.width = info_msg->width;
        intrinsic_.height = info_msg->height;
        intrinsic_.ppx = info_msg->K[2];
        intrinsic_.ppy = info_msg->K[5];
        intrinsic_.fx = info_msg->K[0];
        intrinsic_.fy = info_msg->K[4];
        intrinsic_.model = RS2_DISTORTION_NONE;
        for (int i = 0; i < info_msg->D.size(); i++)
        {
            intrinsic_.coeffs[i] = info_msg->D[i];
        }
        
        for(int i = 0; i < boxes.bounding_boxes.size(); i++)
        {
            if (boxes.bounding_boxes[i].probability > 0.5)
            {
                int center_x, center_y;
                std::string obj_name = boxes.bounding_boxes[i].Class;
                int obj_id = boxes.bounding_boxes[i].id;
                center_x = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin) / 2;
                center_y = (boxes.bounding_boxes[i].ymax + boxes.bounding_boxes[i].ymin) / 2;
                
                int depth_in_mm = image.at<short int>(cv::Point(center_x, center_y));
                
                float pixel[] = {(float)center_x, (float)center_y};
                float point[3];
                float depth_in_m = image.at<short int>(cv::Point(center_x, center_y));
                rs2_deproject_pixel_to_point(&point[0], &intrinsic_, &pixel[0], depth_in_m / 1000);
                cv::Point3d obj_xyz(point[2], -point[0], -point[1]);
                ros::Time current_time = ros::Time::now();
            
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
                
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "camera_link";
                odom_trans.child_frame_id = obj_name + std::to_string(i);
                odom_trans.transform.translation.x = obj_xyz.x;
                odom_trans.transform.translation.y = obj_xyz.y;
                odom_trans.transform.translation.z = obj_xyz.z;
                odom_trans.transform.rotation = odom_quat;
                // ROS_INFO("%s published in %f, %f, %f\n", obj_name.c_str(), obj_xyz.x, obj_xyz.y, obj_xyz.z);
                tf_publisher_.sendTransform(odom_trans);
            }
        }
    }
    void boundingCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &boxmsg)
    {
        boxes = *boxmsg;
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker");
    ObjectDrawer a;
    
    ros::spin();
}