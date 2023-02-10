#include <iostream>
#include <ros/ros.h>
#include <string>
#include <array>
#include <cmath>
#include <pcl/type_traits.h>
#include<pcl_conversions/pcl_conversions.h>
#include <std_msgs/UInt8.h> //builtin_uint8 or UInt8
#include <std_msgs/Float64MultiArray.h>
#include<sensor_msgs/PointCloud2.h>

#include "DBSCAN.h"
#include "OBB.h"

ros::Publisher client_target_pose;
ros::Publisher client_command;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>& point_cloud = *pc_ptr;

void convertPointCloud2Msgs(const sensor_msgs::PointCloud2 msgs){
    pcl::fromROSMsg(msgs, point_cloud);
}

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
    ros::Subscriber pointCloudSubscriber = n.subscribe("/sensor/pointclouds", 10, convertPointCloud2Msgs);
    // let us dance
    ros::spin();
    return 0;
}

void analyseFeedback(const std_msgs::UInt8& state){
    std::cout << "[INFO] Feedback is " << state << std::endl;
    unsigned int  s = state.data;
    switch (s)
    {
    case 1:
    {
        std::cout << "The robot is ready, fetching pointclouds data" << std::endl;
        std::vector<pcl::PointXYZ> upper_mat;
        upper_mat.reserve(256);
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_cloud.begin(); it != point_cloud.end(); it++) {
            if (it->z < 0.91) {
                upper_mat.push_back(*it);
            }
        }
        //TODO Clustering
        std::vector<int> point_label;
        uint group_num = DBSCAN(upper_mat, point_label, 0.2, 4);
        std::cout << "Found " << group_num << " boxes." << std::endl;

        if (group_num == 0){
            std_msgs::UInt8 command_msg;
            command_msg.data = 1; //Everything is done
            client_command.publish(command_msg);
            break;
        }

        //TODO OBB
        std::vector<pcl::PointXYZ> aPiece;
        aPiece.reserve(256);
        for (size_t i = 0; i < upper_mat.size(); i++) {
            if (point_label[i] == 1) {
                aPiece.push_back(upper_mat[i]);
            }
        }
        OBB_Descriptor descriptor;
        OBB obb_solver(aPiece);
        obb_solver.compute(descriptor);

    // finnaly
        Eigen::Vector3f pose_z; pose_z << 0, 0, 1;
        Eigen::Vector3f pose_y; pose_y << descriptor.main_vector_x, descriptor.main_vector_y, 0;
        Eigen::Vector3f pose_x; pose_x = pose_y.cross(pose_z);
        std_msgs::Float64MultiArray pose_msg;
        pose_msg.data = std::vector<double>{
            pose_x(0),pose_y(0), pose_z(0),descriptor.center_x,
            pose_x(1),pose_y(1), pose_z(1),descriptor.center_y,
            pose_x(2),pose_y(2), pose_z(2),descriptor.center_z };
        client_target_pose.publish(pose_msg);
        break;
    }
    default:
        std::cout << "Unknown state: "<< s << std::endl;
        break;
    }
}