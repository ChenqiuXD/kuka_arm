#ifndef TARGET_POSE_UTILS_H
#define TARGET_POSE_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>

tf::StampedTransform getTargetTrans();
geometry_msgs::Pose setTarget(tf::StampedTransform targetTrans);
void rotateTargetPose(Eigen::Quaterniond q, geometry_msgs::Pose* pose);

#endif