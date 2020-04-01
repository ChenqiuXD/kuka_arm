#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh_;
    
    ros::Publisher pub = nh_.advertise<geometry_msgs::Pose>("/kuka_arm/target_pos", 10);
    ros::Rate loopRate(10);

    int count = 0;

    while(ros::ok()){
        geometry_msgs::Pose pos;
        if(count % 10 <5){
            pos.position.x = 1.0;
            pos.orientation.w = 1.0;
        } else{
            pos.position.x = 2.0;
            pos.orientation.w = 2.0;
        }

        count += 1;
        if(count >= 255)    count = 0;
        pub.publish(pos);
        ros::spinOnce();
        loopRate.sleep();
    }
}