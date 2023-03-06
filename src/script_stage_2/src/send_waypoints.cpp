#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

void read_csv(std::string file_name, std::vector<std::vector<double>>& output);

int main(int argc, char* argv[]){
        std::string path_to_csv_file = "src/script_stage_2/traj.csv";
        std::vector<std::vector<double>> waypoints;
        read_csv(path_to_csv_file, waypoints);
        ros::init(argc, argv, "ros_client");
        ros::NodeHandle n;
        ros::Publisher add_new_waypoints;
        add_new_waypoints = n.advertise<std_msgs::Float64MultiArray>("/waypoints", 10);
        size_t waypoints_num = waypoints.size();
        std::cout << "[Info] Load  " << waypoints_num << " waypoints.\n" ;
        ros::Rate r(10);
         while (true){
                        for(size_t i=0; i< waypoints_num; i++){
                std::vector<double> waypoint = waypoints[i];
                if (waypoint.size() != 6){
                        std::cout << "[Error] The robot is 6 DOF while this vector length is " <<waypoint.size() << ".\n" ;
                }
                std_msgs::Float64MultiArray new_msgs;
                new_msgs.data = waypoint;
                add_new_waypoints.publish(new_msgs);
                r.sleep();
        }
         }

        ros::spin();
        return 0;
}

void read_csv(std::string file_name, std::vector<std::vector<double>>& output){
        output.clear();
        std::fstream fs;
        fs.open(file_name, std::fstream::in);
        if (!fs.is_open())
        {
                std::cout << "Error: opening file fail" << std::endl;
                std::exit(1);
        }
        std::istringstream input_str_stream;
        std::string a_line;
        std::vector<double> nums;
        std::string a_cell;
        while (std::getline(fs, a_line)){
                nums.clear();
                input_str_stream.clear();
                input_str_stream.str(a_line);
                while (std::getline(input_str_stream, a_cell, ',')){
                        nums.push_back(atof(a_cell.c_str()));
                        }
                output.push_back(nums);
        }
}