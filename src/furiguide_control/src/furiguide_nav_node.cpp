#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include "furiguide_nav_node.h"

#define correctAngle(X, L) X > 0 ? X : X + L

ros::Publisher tag_pose_pub;
ros::Publisher steer_twist_pub;

geometry_msgs::PoseWithCovariance current_pose;
geometry_msgs::PoseStamped current_waypoint_pose;
int current_waypoint_id = -1;
bool hasWaypoint = false;

// Populated by the tf listener
tf2_ros::Buffer tfBuffer;

double shortestAngle(double A, double B) {
  double diff = A - B;

  if (diff > M_PI)
    return diff - 2*M_PI;
  else if (diff < -M_PI)
    return diff + 2*M_PI;

  return diff;
}

void processAprilTag(const apriltags_ros::AprilTagDetection& tag) {
  ROS_INFO("Processing Waypoint: %d", tag.id);

  // ignore extraneous waypoints (avoid backtracking or seeing too far)
  if (tag.id < current_waypoint_id || tag.id > current_waypoint_id + 1) {
    return; // ignore this tag
  }

  // look for the next waypoint
  if (tag.id == current_waypoint_id + 1) {
    current_waypoint_id++;
  }

  geometry_msgs::TransformStamped transformStamped;

  // look up the transform of the camera from tf2, then store the pose relative to the base_link
  // be careful to get the transform at the time of tag detection for accurate positioning information
  // this effect is especially bad in simulation when manually positioning the robot
  try{
    transformStamped = tfBuffer.lookupTransform("odom", "camera_link_optical", tag.pose.header.stamp);
    tf2::doTransform(tag.pose, current_waypoint_pose, transformStamped);
    hasWaypoint = true;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void steerToWaypoint() {
  geometry_msgs::Twist twist;
  
  ROS_INFO("Correcting robot angle to waypoint %d", current_waypoint_id);
  // TODO: use something more sophitisticated to correct and drive forward
  
  tf2::Quaternion odomQuat, waypointQuat;
  tf2::fromMsg(current_pose.pose.orientation, odomQuat);
  tf2::fromMsg(current_waypoint_pose.pose.orientation, waypointQuat);

  double waypointYaw = correctAngle(tf2::impl::getYaw(waypointQuat), 2*M_PI);
  double odomYaw = correctAngle(tf2::impl::getYaw(odomQuat), 2*M_PI);

  double dPos = tf2::tf2Distance2(posVec, wayVec);
  double dTheta = 0;
  dTheta = -1 * shortestAngle(odomYaw, waypointYaw); // the differential drive controller in gazebo has negative as CW (???)
  
  ROS_INFO("Tag: %f, Odom: %f, Angle: %f", waypointYaw, odomYaw, dTheta);
  twist.linear.x = 0.05; 
  twist.angular.z = dTheta * 0.5;

  steer_twist_pub.publish(twist);
}

// Subscriber Callbacks
void tagDetection_cb(const apriltags_ros::AprilTagDetectionArray& msg) {
  for (int i=0; i < msg.detections.size(); i++) {
    processAprilTag(msg.detections[i]);
  }
}

void odometry_cb(const nav_msgs::Odometry& msg) {
    current_pose = msg.pose;

    if (hasWaypoint)
      steerToWaypoint();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "furiguide_nav_node");

  ros::NodeHandle nh;
  ros::Subscriber tagSub = nh.subscribe("tag_detections", 10, tagDetection_cb);
  ros::Subscriber odomSub = nh.subscribe("odom", 10, odometry_cb);
  tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tag_pose", 1);
  steer_twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();

    tag_pose_pub.publish(current_waypoint_pose);
    r.sleep(); 
  }
}
