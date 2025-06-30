#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

class OdomToTF{

private:
  
  tf::TransformBroadcaster tf_br;
  ros::Subscriber tf_sub;
  std::string root_frame;
  std::string child_frame;  

public:

    OdomToTF(){
        ros::NodeHandle n_private("~"); 
        n_private.getParam("root_frame",root_frame);
        ROS_INFO("Root frame: %s", root_frame.c_str());

        
        n_private.getParam("child_frame",child_frame);
        ROS_INFO("Child frame: %s", child_frame.c_str());

        tf_sub = n_private.subscribe("/input_odom",10,&OdomToTF::tfCallBack, this);
    }

void tfCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    // Create tf transform object
    tf::Transform transform;

    // Set origin (translation)
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

    // Set rotation (quaternion)
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    transform.setRotation(q);

    // Publish tf message
    tf_br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, root_frame, child_frame));
}

};

int main(int argc, char **argv){
    
    ros::init(argc, argv, "odom_to_tf");
    OdomToTF odom_to_tf;
    ros::spin();
    return 0;
};


