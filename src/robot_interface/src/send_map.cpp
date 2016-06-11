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
  std::ofstream myfile ("/home/nithin/catkin_ws/src/init_prog/src/receivedMap.map", std::ios::app);
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
	ros::init(argc, argv, "sendMap");
	ros::NodeHandle nh;

    ros::Publisher send_map_pub = nh.advertise<std_msgs::String>("/load_map", 1000);
    ros::Subscriber get_map_sub = nh.subscribe("/load_map", 1000,recordMapCB);

    std_msgs::String msg;
    //ros::Rate loop_rate(10);

	while(ros::ok())
	{
        std::string line;
        std::ifstream myfile ("/home/nithin/catkin_ws/src/init_prog/src/1.map");
        if (myfile.is_open())
        {
            std::cout<<"Sending map... \n";
        while (std::getline (myfile,line) )
            {
              std::stringstream ss;
              ss << line<<'\n';
              msg.data = ss.str();
              send_map_pub.publish(msg);

              ros::spinOnce();

              //loop_rate.sleep();
            }
        myfile.close();
        std::cout<<"Map sent to robot. \n Exiting...\n";
        ros::shutdown();
        exit(1);
        }

        else
        {
            std::cout << "Unable to open file \n";
            std::cout << "Exiting \n";
            ros::shutdown();
            exit(1);
        }
    }

	return 1;
}
