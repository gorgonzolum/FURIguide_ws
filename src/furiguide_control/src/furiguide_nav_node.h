#pragma once

void processOdometry(const nav_msgs::Odometry& msg); 
void processAprilTag(const apriltags_ros::AprilTagDetection& tag); 
void steerToWaypoint(); 
void tagDetection_cb(const apriltags_ros::AprilTagDetectionArray& msg); 
