#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <armadillo>

class GPSToOdometry {
  // Initialize node handle
private:
  ros::NodeHandle n;
  ros::Subscriber gps_sub;
  ros::Publisher odom_pub;
  double lat_r, lon_r, alt_r;
  double lam_r, phi_r, h_r;
  double x_r, y_r, z_r;
  //constants:
  double a = 6378137;
  double b = 6356752.3142;
  double e2 = 1 - pow((b/a),2);

  double old_enu_x = 0, old_enu_y = 0, old_enu_r = 0;

public:
  GPSToOdometry() {    
    // Get reference point from parameters

    n.getParam("lat_r", lat_r);
    n.getParam("lon_r", lon_r);
    n.getParam("alt_r", alt_r);
    //lets transform them into ECEF (check Samuel Drake paper)
    //https://www.researchgate.net/profile/Samuel-Drake/publication/27253833_Converting_GPS_coordinates_phi_lambda_h_to_navigation_coordinates_ENU/links/00b7d516ca79c24fdb000000/Converting-GPS-coordinates-phi-lambda-h-to-navigation-coordinates-ENU.pdf
    
    //bring everything to radians
     phi_r = lat_r*M_PI/180; //latitude
     lam_r = lon_r*M_PI/180; //longitude
     h_r = alt_r;          //altitude
    
    double tmp_r = sqrt(1-e2*pow(sin(phi_r),2));

     x_r = (a/tmp_r+h_r)*cos(phi_r)*cos(lam_r);
     y_r = (a/tmp_r+h_r)*cos(phi_r)*sin(lam_r);
     z_r = (a*(1-e2)/tmp_r+h_r)*sin(phi_r);

    // Subscribe to fix topic: GPS bag data comes on this topic
    gps_sub = n.subscribe("fix", 10, &GPSToOdometry::gpsCallback, this);
    // Publish odometry topic: these will be used by the other nodes
    odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10); //buffer of 10 beacuse why not

  }

private:
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Convert Sat to ECEF
    double ecef_x, ecef_y, ecef_z;
    gpsToEcef(msg->latitude, msg->longitude, msg->altitude, ecef_x, ecef_y, ecef_z);

    // Convert ECEF to ENU
    double enu_x, enu_y, enu_z;
    ecefToEnu(ecef_x, ecef_y, ecef_z, enu_x, enu_y, enu_z);
    
    // Create odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "gps_odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = enu_x;
    odom.pose.pose.position.y = enu_y;
    odom.pose.pose.position.z = enu_z;

    ROS_INFO("%f", odom.pose.pose.position.x);
    ROS_INFO("%f", odom.pose.pose.position.y);
    ROS_INFO("%f", odom.pose.pose.position.z);

    // Estimate robot heading from consecutive poses
    double Dx = enu_x - old_enu_x;
    double Dy = enu_y - old_enu_y;
    double yaw = atan(Dy/Dx) * M_PI/180;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    // Publish odometry message
    odom_pub.publish(odom);
  }

  // Helper functions for coordinate conversions
  
  void gpsToEcef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z) {
    
    //constants:
    double a = 6378137;
    double b = 6356752.3142;
    double e2 = 1 - pow((b/a),2);
    //bring everything to radians
    double phi = lat*M_PI/180; //latitude
    double lam = lon*M_PI/180; //longitude
    double h = alt;          //altitude
    
    double tmp = sqrt(1-e2*pow(sin(phi),2));

    ecef_x = (a/tmp+h)*cos(phi)*cos(lam);
    ecef_y = (a/tmp+h)*cos(phi)*sin(lam);
    ecef_z = (a*(1-e2)/tmp+h)*sin(phi);
  }
  
  void ecefToEnu(double x, double y, double z, double& enu_x, double& enu_y, double& enu_z) {
    arma::mat A = {{-sin(lam_r), cos(lam_r), 0}, {-sin(phi_r)*cos(lam_r), -sin(phi_r)*sin(lam_r), cos(phi_r)}, {cos(phi_r)*cos(lam_r), cos(phi_r)*sin(lam_r), sin(phi_r)}};
    arma::vec v = {x-x_r, y-y_r, z-z_r};
    arma::vec result = A * v;
    enu_x = result(0);
    enu_y = result(1);
    enu_z = result(2);
    //sperando che armadillo funzioni correttamente
  }
  
  
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_to_odom");
  GPSToOdometry gps_to_odom;
  
  ros::spin();
  return 0;
}