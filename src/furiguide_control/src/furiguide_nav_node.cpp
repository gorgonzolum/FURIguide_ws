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

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "furiguide_nav_node.h"

#define correctAngle(X, L) X > 0 ? X : X + L

ros::Publisher tag_pose_pub;
ros::Publisher steer_twist_pub;

geometry_msgs::PoseWithCovariance current_pose;
geometry_msgs::PoseStamped current_waypoint_pose;
int current_waypoint_id = -1;

bool arrivedAtWaypoint = false;
bool hasWaypoint = false;
bool updateGoal = false;

// Populated by the tf listener
tf2_ros::Buffer tfBuffer;

// actionlib alias
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal nav_goal;

double shortestAngle(double A, double B) {
  double diff = fmod(A - B, 2*M_PI);

  if (diff > M_PI)
    return diff - 2*M_PI;
  else if (diff < -M_PI)
    return diff + 2*M_PI;

  return diff;
}

void processAprilTag(const apriltags_ros::AprilTagDetection& tag) {
  //ROS_INFO("Processing Waypoint: %d", tag.id);

  // ignore extraneous waypoints (avoid backtracking or seeing too far)
  if (tag.id < current_waypoint_id || tag.id > current_waypoint_id + 1) {
    return; // ignore this tag
  }

  geometry_msgs::TransformStamped transformStamped;

  // look up the transform of the camera from tf2, then store the pose relative to the base_link
  // be careful to get the transform at the time of tag detection for accurate positioning information
  // this effect is especially bad in simulation when manually positioning the robot
  try{
    transformStamped = tfBuffer.lookupTransform("odom", "camera_link_optical", tag.pose.header.stamp);
    tf2::doTransform(tag.pose, current_waypoint_pose, transformStamped);
    arrivedAtWaypoint = false;
    hasWaypoint = true;
    //ROS_INFO("way Quat: %f, %f, %f, %f", current_waypoint_pose.pose.orientation.x, current_waypoint_pose.pose.orientation.y, current_waypoint_pose.pose.orientation.z, current_waypoint_pose.pose.orientation.w); 

    // look for the next waypoint
    if (tag.id == current_waypoint_id + 1) {
      current_waypoint_id++;
      updateNavGoal();
      updateGoal = true;
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void steerToWaypoint() {
  geometry_msgs::Twist twist;
  
  //ROS_INFO("Correcting robot angle to waypoint %d", current_waypoint_id);
 
  // translate geometry_msgs into tf2 datatypes
  tf2::Quaternion odomQuat, waypointQuat;
  tf2::Vector3 posVec, wayVec, diffVec;

  tf2::fromMsg(current_pose.pose.orientation, odomQuat);
  tf2::fromMsg(current_waypoint_pose.pose.orientation, waypointQuat);
  tf2::convert(current_pose.pose.position, posVec);
  tf2::convert(current_waypoint_pose.pose.position, wayVec);
  diffVec = wayVec - posVec;

  double waypointYaw = correctAngle(tf2::impl::getYaw(waypointQuat), 2*M_PI);
  double odomYaw = correctAngle(tf2::impl::getYaw(odomQuat), 2*M_PI);

  double dPos = tf2::tf2Distance2(posVec, wayVec);
  double dTheta = 0;
  if (dPos < 0.2 || arrivedAtWaypoint) { // match waypoint heading
    dTheta = shortestAngle(odomYaw, waypointYaw);
    arrivedAtWaypoint = true;
  } else { // approach waypoint
    double tagHeading = atan2(diffVec.y(), diffVec.x());
    dTheta = shortestAngle(odomYaw, tagHeading);
    //ROS_INFO("Drive to Position: %f, %f, %f", (wayVec - posVec).x(), (wayVec - posVec).y(), (wayVec-posVec).z());
    //ROS_INFO("odom Quat: %f, %f, %f", odomQuat.getAxis().x(), odomQuat.getAxis().y(), odomQuat.getAxis().z());
  }
  
  // Correct for angle and drive forward
  //ROS_INFO("Tag: %f, Odom: %f, dPos: %f, Angle: %f", waypointYaw, odomYaw, dPos, dTheta);
  twist.linear.x = 0.5; 
  twist.angular.z = -1 * dTheta * 0.8; // ROS yaw is CCW increasing, MUST * -1 to account for this 

  //steer_twist_pub.publish(twist);
/*
  ac.waitForResult()

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Successfully arrived at goal");
  else
    ROS_INFO("Failed to arrive at goal...");
*/
}

void updateNavGoal() {
  /* send a goal coordinate to the nav stack */
  tf2::Quaternion odomQuat, waypointQuat;
  tf2::Vector3 posVec, wayVec, projVec;

  // FIXME: project this onto the xy plane (?)
  tf2::fromMsg(current_waypoint_pose.pose.orientation, waypointQuat);
  tf2::convert(current_pose.pose.position, posVec);
  tf2::convert(current_waypoint_pose.pose.position, wayVec);

  projVec = tf2::quatRotate(waypointQuat, tf2::Vector3(1, 0, 0)).normalize();

  nav_goal.target_pose.header.frame_id = "odom";
  nav_goal.target_pose.header.stamp = ros::Time::now();

  nav_goal.target_pose.pose.position.x = wayVec.x() + 4.0 * projVec.x();
  nav_goal.target_pose.pose.position.y = wayVec.y() + 4.0 * projVec.y();

  nav_goal.target_pose.pose.orientation.x = waypointQuat.x();
  nav_goal.target_pose.pose.orientation.y = waypointQuat.y();
  nav_goal.target_pose.pose.orientation.z = waypointQuat.z();
  nav_goal.target_pose.pose.orientation.w = waypointQuat.w();

  ROS_INFO("proj Vec: %f, %f, %f", projVec.x(), projVec.y(), projVec.z());
  ROS_INFO("goal Pos: %f, %f, %f", nav_goal.target_pose.pose.position.x, nav_goal.target_pose.pose.position.y, nav_goal.target_pose.pose.position.z);
  ROS_INFO("goal Quat: %f, %f, %f, %f", waypointQuat.x(), waypointQuat.y(), waypointQuat.z(), waypointQuat.w()); 
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

  // spin an action client thread up
  MoveBaseClient ac("move_base", true);

  // wait for the move_base action server
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();

    if (updateGoal) {
      ROS_INFO("Sending Goal");
      ac.sendGoal(nav_goal);
      updateGoal = false;
    }

    tag_pose_pub.publish(current_waypoint_pose);
    r.sleep(); 
  }
}
