#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <iostream>

int main(int argc, char** argv){

    ros::init(argc, argv, "selecting_menu");

    ros::NodeHandle node("~");

    ros::Publisher selector_pub = node.advertise<std_msgs::UInt16>("/selector", 100);

    while(ros::ok()){

    std::cout<< "Choose control algorith:" << std::endl;
    std::cout<< "0 - Dummy \n 1 - Voyager \n 2 - Wallfollower \n";
    std_msgs::UInt16 choice;
    std::cin >> choice.data;

    selector_pub.publish(choice);
    }

    ros::spin();

    return 0;
}
