#include <iostream>
#include <ros/ros.h>
#include <array>
#include <std_msgs/String.h>
#include <string>

void printUR5JointsInfo(){
    std::cout << "UR5 Joints Info :" << std::endl;
}

void analyseFeedback(const std_msgs::String& state){
    std::cout << "[INFO] Feedback is " << state << std::endl;
}

int main(int argc, char** argv){
    std::cout << "Here is a simple demo for ROS and CoppeliaSim" << std::endl;

    // ros ndoe init
    ros::init(argc, argv, "ros_client");
    ros::NodeHandle n;

    //-- node commander
    ros::Publisher commander = n.advertise<std_msgs::String>("/sim/command", 10);
    ros::Subscriber state_feedback = n.subscribe("/sim/state", 10, analyseFeedback);
    
    // let us dance
    ros::Rate loop_rate(10);
    while (ros::ok()){
        std_msgs::String command;
        command.data = "shoot";
        commander.publish(command);
        loop_rate.sleep();
    }

    return 0;

}