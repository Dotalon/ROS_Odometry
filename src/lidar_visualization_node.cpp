#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <first_project/lidar_visualization_nodeConfig.h>

class LidarVisualizationNode {
private:
  ros::NodeHandle n;
  ros::Subscriber points_sub;
  ros::Publisher points_pub; 
  std::string target_frame;
  dynamic_reconfigure::Server<first_project::lidar_visualization_nodeConfig> server;
  dynamic_reconfigure::Server<first_project::lidar_visualization_nodeConfig>::CallbackType f;

public:
  LidarVisualizationNode() {
    target_frame = "wheel_odom"; // Initialize the target frame

    f = boost::bind(&LidarVisualizationNode::callback, this, _1, _2);
    server.setCallback(f);

    points_sub = n.subscribe("os_cloud_node/points", 10, &LidarVisualizationNode::pointsCallback, this);
    points_pub = n.advertise<sensor_msgs::PointCloud2>("output_topic", 10); 
  }

  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2 msg_nonconst = *msg;
    msg_nonconst.header.frame_id = target_frame;
    points_pub.publish(msg_nonconst);
  }
  
  void callback(first_project::lidar_visualization_nodeConfig &config, uint32_t level) {
    target_frame = config.target_frame;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_visualization_node");
  LidarVisualizationNode lidar_visualization_node;
  
  ros::spin();
  return 0;
}