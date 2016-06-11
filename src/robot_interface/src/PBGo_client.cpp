#include "ros/ros.h"
#include "robot_interface/PBGo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PBGo_client");
  ROS_INFO("Please Enter the goal you would like to go to. \n Example: \n goal1       or \n home       or \n dock : \n \n \n Where would you like to go : ");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robot_interface::PBGo>("PBGo");
  robot_interface::PBGo srv;
  //std::string req;
  std::cin>>srv.request.goal_name;
  //srv.request.goal_nameatoll(argv[1]);
  if (client.call(srv))
  {     std::cout<<"Sending target";
	while (1)
		if (srv.response.PB_reached.c_str()=="Reached")
			break;
        if (srv.response.PB_reached.c_str()=="Reached")
            ROS_INFO("Robot has reached the destination..");
        else
            ROS_INFO("Robot failed to reach destination!!");
  }
  else
  {
    ROS_ERROR("Failed to call service PBGo");
    return 1;
  }

  return 0;
}

