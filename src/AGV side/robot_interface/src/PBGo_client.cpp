////////////////////////////////////// Nithin ///////////////////////////////////////////////////////


/* This is a ROS client for the PBGo_server
 * It connects to the PBGo_server and makes the robot go to the given goal
 * Must be run after PBGo_server 
 * User can mention goals as named on the map loaded on the server
 * End of function, the server returns a success value which ends this client function
 */

////////////////////////////////////////////////////////////////////////////////////////////////////

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
  std::cin>>srv.request.goal_name;
  //srv.request.goal_name= atoll(argv[1]); // Add this to read from commandline i.e. 
					//rosrun robot_interface PBGo_client goal2
  if (client.call(srv))
  {     std::cout<<"Sending target";
	while (1)
		if (srv.response.PB_reached)
			break;
        if (srv.response.PB_reached)
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

