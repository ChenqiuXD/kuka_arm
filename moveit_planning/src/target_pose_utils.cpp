#include "target_pose_utils.h"


using namespace std;

tf::StampedTransform getTargetTrans()
{
    tf::TransformListener listener;
    int TF_TIMEOUT = 1;

    // Listen to the tf message to get the target pose transform published by imgProcess node
    tf::StampedTransform transform;
    listener.waitForTransform("/base_link", "/target_pos", ros::Time(0), ros::Duration(TF_TIMEOUT*5), ros::Duration(TF_TIMEOUT/3));
    try{
        listener.lookupTransform("/base_link", "/target_pos", ros::Time(0), transform);
        ROS_INFO("Successfully listen from the tf listener");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

geometry_msgs::Pose setTarget(tf::StampedTransform targetTrans)
{
    geometry_msgs::Pose target_pose;
    // Get the rotation
    Eigen::Quaterniond q(targetTrans.getRotation().x(),
                         targetTrans.getRotation().y(),
                         targetTrans.getRotation().z(),
                         targetTrans.getRotation().w());
    cout << "The rotation of target is: \n" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl; 
    rotateTargetPose(q, &target_pose);

    // Get the translation
    target_pose.position.x = targetTrans.getOrigin().x();
    target_pose.position.y = targetTrans.getOrigin().y();
    target_pose.position.z = targetTrans.getOrigin().z()+0.08;
    return target_pose;
}

void rotateTargetPose(Eigen::Quaterniond q, geometry_msgs::Pose* pose)
{
    // Rotate the target quaternion so that the gripper could grasp the target from top to bottom
    Eigen::Matrix3d tarRot = q.toRotationMatrix();

    // Rotate the matrix, axis: x, angle: 90 degree
    Eigen::Matrix3d rotAxisX;
    rotAxisX << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;

    // Get the target pose
    Eigen::Matrix3d result = rotAxisX * tarRot;
    Eigen::Matrix3d qMat;
    qMat << result(0,0), result(0,1), result(0,2),
            result(1,0), result(1,1), result(1,2),
            result(2,0), result(2,1), result(2,2);
    Eigen::Quaterniond qResult;
    qResult = qMat;
    pose->orientation.x = qResult.x();
    pose->orientation.y = qResult.y();
    pose->orientation.z = qResult.z();
    pose->orientation.w = qResult.w();
    cout << "The rotated quaternion is: (x, y, z, w)\n" << qResult.x() << " " << qResult.y() << " " << qResult.z() << " " << qResult.w() << endl;
}