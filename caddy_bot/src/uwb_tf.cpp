#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

#define FALSE (0)
#define TRUE (!FALSE)

typedef struct {
	float* buffer;
	float sum;
	uint32_t length;
	uint32_t currentIdx;
}ST_MovingAverage;

int32_t MovingAverageInit(ST_MovingAverage* movingAverage, uint32_t length)
{
	int32_t result;

	movingAverage->currentIdx = 0;
	movingAverage->sum = 0;
	movingAverage->length = length;

	movingAverage->buffer = (float*)malloc(length * sizeof(float));

	if (movingAverage->buffer == NULL) {
		result = FALSE;
	}
	else {
		memset(movingAverage->buffer, 0, length * sizeof(float));
		result = TRUE;
	}

	return result;
}

void MovingAverageDelete(ST_MovingAverage* movingAverage)
{
	free(movingAverage->buffer);
}

float MovingAverage(ST_MovingAverage* movingAverage, float value)
{
	float result;

	movingAverage->sum -= movingAverage->buffer[movingAverage->currentIdx];
	movingAverage->sum += value;
	movingAverage->buffer[movingAverage->currentIdx] = value;

	movingAverage->currentIdx++;
	movingAverage->currentIdx %= movingAverage->length;

	result = movingAverage->sum / movingAverage->length;

	return result;
}

typedef struct Position {
    double x;
    double y;
} Position;

class UwbFollow
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_main_;
    ros::Subscriber sub_b_;
    ros::Subscriber sub_c_;
    ros::Subscriber sub_odom_;
    tf::TransformBroadcaster tf_publisher_;
    Position position_self = {0,0};
    Position position_B = {0.425,-0.55};
    Position position_C = {-0.425,-0.55};
    float raw_self;
    float raw_b;
    float raw_c;
    float range_self;
    float range_B;
    float range_C;
    float x;
    float y;
    int32_t InitResult = 0;
    ST_MovingAverage movingAverage[3];
    float angular = 0.0;
public:
    UwbFollow()
    {
        sub_main_ = nh_.subscribe("/uwb_main_range", 256, &UwbFollow::mainCallback, this);
        sub_b_ = nh_.subscribe("/uwb_b_range", 256, &UwbFollow::bCallback, this);
        sub_c_ = nh_.subscribe("/uwb_c_range", 256, &UwbFollow::cCallback, this);
        sub_odom_ = nh_.subscribe("/odometry/filtered", 256, &UwbFollow::odomCallback, this);
        
        InitResult = MovingAverageInit(&movingAverage[0], 15);
        if (InitResult == FALSE) {
            ROS_INFO("Fail MovingAverage[0] Init");
        }
        InitResult = MovingAverageInit(&movingAverage[1], 15);
        if (InitResult == FALSE) {
            ROS_INFO("Fail MovingAverage[1] Init");
        }
        InitResult = MovingAverageInit(&movingAverage[2], 15);
        if (InitResult == FALSE) {
            ROS_INFO("Fail MovingAverage[2] Init");
        }
    }
    void mainCallback(const std_msgs::Float32 &message)
    {
        if(message.data > 0.2){
            raw_self = message.data;
            range_self = MovingAverage(&movingAverage[0], message.data);
        }
    }
    void bCallback(const std_msgs::Float32 &message)
    {
        if(message.data > 0.2){
            raw_b = message.data;
            range_B = MovingAverage(&movingAverage[1], message.data);
        }
    }
    void cCallback(const std_msgs::Float32 &message)
    {
        if(message.data > 0.2){
            raw_c = message.data;
            range_C = MovingAverage(&movingAverage[2], message.data);
        }
        
        double A = ( (-2*position_self.x) + (2*position_B.x) );
        double B = ( (-2*position_self.y) + (2*position_B.y) );
        double C = (range_self*range_self) - (range_B*range_B) - (position_self.x*position_self.x) + (position_B.x*position_B.x) - (position_self.y*position_self.y) + (position_B.y*position_B.y);
        double D = ( (-2*position_B.x) + (2*position_C.x) );
        double E = ( (-2*position_B.y) + (2*position_C.y) );
        double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

        x = (C*E-F*B) / (E*A-B*D);
        y = (C*D-A*F) / (B*D-A*E);
        ROS_INFO("ran M : %.2f, B : %.2f, C : %.2f, X : %.2f, Y : %.2f",range_self,range_B,range_C,x,y);

        try
        {
            ros::Time current_time = ros::Time::now();
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "uwb_link";
            odom_trans.child_frame_id = "person";
            float temp_x, temp_y;
            if (angular >=0){
                temp_x = x * cos(angular) + y * sin(angular);
                temp_y = y * cos(angular) - x * sin(angular);
            }else{
                temp_x = x * cos(angular) + y * sin(angular);
                temp_y = y * cos(angular) - x * sin(angular);                
            }
            odom_trans.transform.translation.x = temp_x;
            odom_trans.transform.translation.y = temp_y;
            // odom_trans.transform.translation.z = ;
            odom_trans.transform.rotation = odom_quat;
            tf_publisher_.sendTransform(odom_trans);
        }
        catch(int expn)
        {
            
        }
    }
    void odomCallback(const nav_msgs::Odometry &message)
    {
        angular = message.twist.twist.angular.z;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker");
    UwbFollow a;
    
    ros::spin();
}