#pragma once

void processAprilTag(const apriltags_ros::AprilTagDetection& tag); 
void steerToWaypoint(); 
void updateNavGoal(); 
void tagDetection_cb(const apriltags_ros::AprilTagDetectionArray& msg); 
void odometry_cb(const nav_msgs::Odometry& msg); 
