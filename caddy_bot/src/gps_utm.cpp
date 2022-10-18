#include <iostream>
#include <ros/ros.h>
#include <iomanip>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <cmath>
#include <vector>
#include <robot_localization/navsat_conversions.h>

class LatLon2UTM{
    public:
    LatLon2UTM();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;    

    void wgs2utm(double lat, double lon, int zone,double& east, double& north); // WGS84 to UTM
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
};

LatLon2UTM::LatLon2UTM() : nh_("~"){
    gps_sub_ = nh_.subscribe("/gps_data", 1, &LatLon2UTM::gps_callback, this);
}

void LatLon2UTM::wgs2utm(double lat, double lon, int zone , double& east, double& north){
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
}

void LatLon2UTM::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg){
    double lat = gps_msg->latitude;
    double lon = gps_msg->longitude;
    double east, north;
    int zone;
    if(lon < 0){
        zone = (lon + 180) / 6 + 1;
    }
    else{
        zone = lon / 6 + 31;
    }
    wgs2utm(lat, lon, zone, east, north);

    double easting = east * 0.9996 + 500000;
    double northing = north * 0.9996;

    std::cout << std::setprecision(9);
    std::cout << "lat : " << lat << std::endl;
    std::cout << "lon : " << lon << std::endl;
    std::cout << "zone : " << zone << std::endl;
    std::cout << "E : " << easting << std::endl;
    std::cout << "N : " << northing << std::endl;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "latlon2utm");
    LatLon2UTM node;
    ros::spin();
    return 0;
}