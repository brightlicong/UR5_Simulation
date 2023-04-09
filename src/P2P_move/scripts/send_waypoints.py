#! /usr/bin/env python
import rospy
import std_msgs.msg as msg

def cell2num(item):
        return float(item.strip())

def read_csv(file_name):
        data = []
        with open(file_name, "r") as f:
                for line in f.readlines():
                        cells = line.split(",")
                        data.append(map(cell2num,cells))
        return data


def main():
        path_to_csv_file = "src/script_stage_2/traj.csv"
        waypoints = read_csv(path_to_csv_file)
        rospy.init_node("ros_client", anonymous=True)
        pub = rospy.Publisher("/waypoints", msg.Float64MultiArray, queue_size=10)  
        rate = rospy.Rate(1) 
        print("[Info] Load  {} waypoints.".format(len(waypoints)))   
        loop_round = 0
        while not rospy.is_shutdown():
                loop_round += 1
                print("[INFO] Loop {}".format(loop_round))
                for wp in waypoints:
                        if len(wp) != 6:
                                print("[Error] The robot is 6 DOF while this vector length is {}".format(len(wp))) 
                        send_msg = msg.Float64MultiArray()  
                        send_msg.data = wp
                        pub.publish(send_msg)    
                        rate.sleep()

if __name__ == "__main__":
        try:
                main()
        except KeyboardInterrupt:
                print("Break down.")
