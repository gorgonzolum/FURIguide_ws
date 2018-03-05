#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include "furiguide_nav_node.h"

ros::Publisher tag_pose_pub;
ros::Publisher steer_twist_pub;
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped current_waypoint_pose;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
void processOdometry(const nav_msgs::Odometry& msg) {
    current_pose = msg.pose.pose;

    steerToWaypoint();
}

void processAprilTag(const apriltags_ros::AprilTagDetection& tag) {
  ROS_INFO("Processing Waypoint");
  current_waypoint_pose = tag.pose;

  geometry_msgs::TransformStamped transformStamped;

  // look up the transform of the camera from tf2, then store the pose relative to the base_link
  try{
    transformStamped = tfBuffer.lookupTransform("camera_link", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void steerToWaypoint() {
  geometry_msgs::Twist twist;
  
  ROS_INFO("Correcting robot angle to waypoint");
  // transform pose camera to odom frame
  // get tf from robot_state_publisher
  // calculate new heading, orient toward apriltag pose with simple proportional controller or PID (or OTHER)

  steer_twist_pub.publish(twist);
}

void tagDetection_cb(const apriltags_ros::AprilTagDetectionArray& msg) {
  for (int i=0; i < msg.detections.size(); i++) {
    processAprilTag(msg.detections[i]);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "furiguide_nav_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tag_detections", 10, tagDetection_cb);
  tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tag_pose", 1);
  steer_twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();

    tag_pose_pub.publish(current_waypoint_pose);
    r.sleep(); 
  }
}
