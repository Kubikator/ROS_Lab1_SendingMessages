#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

bool done = 0;
int count = 0;
int click_count = -1;
int size = 4;
move_base_msgs::MoveBaseGoal *goals;

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
//умный указатель на объект - клиент
boost::shared_ptr<MoveBaseClient> moveBaseClientPtr;

void resize()
{
	int new_size = size * 2;
	move_base_msgs::MoveBaseGoal *new_goals = new move_base_msgs::MoveBaseGoal[new_size];
	for (int i = 0; i < count; ++i)
		new_goals[i] = goals[i];

	delete[] goals;
	goals = new_goals;
	size = new_size;
}

void targetAngle(int i)
{
	if (i >= 0)
	{
		if (count > 1)
		{
			double x0 = goals[i].target_pose.pose.position.x;
			double y0 = goals[i].target_pose.pose.position.y;
			double x1, y1;
			if(i == count - 1)
			{
				x1 = goals[0].target_pose.pose.position.x;
				y1 = goals[0].target_pose.pose.position.y;
			}
			else
			{	
				x1 = goals[i + 1].target_pose.pose.position.x;
				y1 = goals[i + 1].target_pose.pose.position.y;
			}
			double l = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
			goals[i].target_pose.pose.orientation.z = (y1 - y0) / l;
			goals[i].target_pose.pose.orientation.w = (x1 - x0) / l;
		}
		else
		{
			goals[i].target_pose.pose.orientation.z = sin(M_PI/2);
			goals[i].target_pose.pose.orientation.w = cos(M_PI/2);
		}
	}
}

void done_callback(const actionlib::SimpleClientGoalState& state,
                   const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Target is reached");
        
    }
    else
    {
        ROS_ERROR("move_base has failed");
    }
    done = 1;
}


void feedback_callback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("feedback "<<
                    " robot pose x" << feedback->base_position.pose.position.x <<
                    " y = "<<feedback->base_position.pose.position.y);
}

void active_callback()
{
    ROS_INFO_STREAM("goal is started");
}



void clickPointCallback(const geometry_msgs::PointStamped& point)
{

	if (count == size) resize();
	
	++count;
	
	click_count = (click_count + 1) % count;

    ROS_INFO_STREAM(" get point "<<point.point.x<<" "<<point.point.y);

    goals[click_count].target_pose.pose.position = point.point;
	targetAngle(click_count);
	targetAngle(click_count - 1);
	goals[click_count].target_pose.header.frame_id = "map";
	goals[click_count].target_pose.header.stamp = ros::Time::now();
    
    if (count == 1) done = 1;
}


void sendGoal(int& waypoint_count)
{
	waypoint_count = (waypoint_count + 1) % count;
	
	done = 0;
	ROS_WARN_STREAM("Point number: " << waypoint_count << " count: "<< count);
	moveBaseClientPtr->sendGoal(goals[waypoint_count],
					done_callback,
					active_callback,
					feedback_callback);
					
}

/* в отчёте написать про amcl, gmapping */

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_node");


    ros::NodeHandle node("~");

    moveBaseClientPtr.reset(new MoveBaseClient("/move_base", false));

    while( !moveBaseClientPtr->isServerConnected())
    {
        ros::spinOnce();
    }

    ROS_INFO_STREAM("server move_base is connected");
    
    goals = new move_base_msgs::MoveBaseGoal[size];

    ros::Subscriber point_sub = node.subscribe("/clicked_point", 1, clickPointCallback);
    
    int waypoint_count = -1;
    while(ros::ok())
    {
    	ros::spinOnce();
    	if (done)
    	{
    	    sendGoal(waypoint_count);
    	}
    }
    
    delete[] goals;

    return 0;
}
