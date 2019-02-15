#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vizzy_fingers/MarkerArray.h>

using namespace std;

void chatterCallback(const vizzy_fingers::MarkerArray::ConstPtr& msg)
{
 
  ROS_INFO("In a frame, I read :\n");
  
  for(int i = 0; i < msg->markers.size(); i++ )
  {   
       ROS_INFO("    ID: %d\n", msg->markers.at(i).id );
       ROS_INFO("    Position:\n" );
       ROS_INFO("       x:%f\n", msg->markers.at(i).pose.pose.position.x );
       ROS_INFO("       y:%f\n", msg->markers.at(i).pose.pose.position.y );
       ROS_INFO("       z:%f\n", msg->markers.at(i).pose.pose.position.z );
       ROS_INFO("    Orientation:\n" );
       ROS_INFO("       x:%f\n", msg->markers.at(i).pose.pose.orientation.x );
       ROS_INFO("       y:%f\n", msg->markers.at(i).pose.pose.orientation.y );
       ROS_INFO("       z:%f\n", msg->markers.at(i).pose.pose.orientation.z );
       ROS_INFO("       w:%f\n\n\n\n", msg->markers.at(i).pose.pose.orientation.w );
  }
 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Markers_subscriber");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("Markers_chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
