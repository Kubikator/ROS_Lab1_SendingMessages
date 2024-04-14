#include "ros/ros.h"
#include "std_msgs/String.h"
#include<sstream>


class Listener{

  public:

  ~Listener(){
    ros::spin();
  }

  void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_ERROR("I heard: [%s]", msg->data.c_str());

    std_msgs::String message;

    std::stringstream ss;
    ss << "Got message!";
    message.data = ss.str();
    pub.publish(message);
  }

  private:

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("reply", 1000);;
  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::chatterCallback, this);;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  Listener li;

  return 0;
}
