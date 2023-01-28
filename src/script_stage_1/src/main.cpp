#include <iostream>
#include <ros/ros.h>
#include <array>
#include <vector>
#include <std_msgs/UInt8.h> //builtin_uint8 or UInt8
#include <std_msgs/Float64MultiArray.h>
#include <string>

ros::Publisher client_target_pose;
ros::Publisher client_command;

void analyseFeedback(const std_msgs::UInt8& state);

int main(int argc, char** argv){
    std::cout << "Here is a simple demo for ROS and CoppeliaSim" << std::endl;
    ros::init(argc, argv, "ros_client");
    // ros ndoe init
    ros::NodeHandle n;
    //-- node commander
    ros::Subscriber state_feedback = n.subscribe("/robot/state", 10, analyseFeedback);
    client_command = n.advertise<std_msgs::UInt8>("client/command", 10);
    client_target_pose  = n.advertise<std_msgs::Float64MultiArray>("client/pose", 12);

    // let us dance
    ros::spin();

    return 0;

}

void analyseFeedback(const std_msgs::UInt8& state){
    std::cout << "[INFO] Feedback is " << state << std::endl;
    unsigned int  s = state.data;
    std_msgs::Float64MultiArray pose_msg;
    std_msgs::UInt8 command_msg;
    switch (s)
    {
    case 1:
        std::cout << "The robot is ready, fetching pointclouds data" << std::endl;
        pose_msg.data = std::vector<double>{1.0, 0.0, 0.0, 0.6, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.12};
        client_target_pose.publish(pose_msg);
        //command_msg.data = 1;
        //client_command.publish(command_msg);
        break;
    
    default:
        std::cout << "Unknown state: "<< s << std::endl;
        break;
    }
}