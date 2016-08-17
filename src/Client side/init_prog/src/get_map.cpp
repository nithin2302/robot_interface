#include "ros/ros.h"
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include "ros/master.h"
#include <std_msgs/String.h>


void recordMapCB(const std_msgs::String::ConstPtr& msg)
{
  std::ofstream myfile ("/home/receivedMap.map", std::ios::app);
  if (myfile.is_open())
  {
    myfile << msg->data.c_str();
    myfile.close();
  }
  else std::cout << "Unable to open file";
}


/* Handles top level control as usual */
int main(int argc, char **argv)
{


  /* initialize the ros node */
	ros::init(argc, argv, "getMap");
	ros::NodeHandle nh;
    ros::Subscriber get_map_sub = nh.subscribe("/load_map", 1000,recordMapCB);

    std_msgs::String msg;
    //ros::Rate loop_rate(10);
    if(!ros::ok())
    ros::shutdown();

	return 1;
}
