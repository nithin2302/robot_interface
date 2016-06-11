#include "ros/ros.h"
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include "ros/master.h"
#define KEYCODE_1 0x31 
#define KEYCODE_2 0X32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_Q 0X71

static struct termios new1, old;

/* initializes the terminal to new input settings */
void initTermios(int echo) 
{
    tcgetattr(0, &old); /* grab old terminal i/o settings */
    new1 = old; /* make new settings same as old settings */
    new1.c_lflag &= ~ICANON; /* disable buffered i/o */
    new1.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &new1); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios() 
{
    tcsetattr(0, TCSANOW, &old);
}

/* quit the program cleanly and close ros */
void quit()
{
	resetTermios(); /* reset the terminal to old settings */
	ros::shutdown(); /*shutdown ros */
	exit(1); /* exit the system */
}

/* Handles top level control as usual */
int main(int argc, char **argv)
{


  /* initialize the ros node */
	ros::init(argc, argv, "interface");
	ros::NodeHandle nh;

  /* change the terminal input settings */ 
	initTermios(0);
	
		std::cout  	
			<< "******************************************************************" << std::endl
			<< "*                      ROBOT INTERFACE                           *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*          Welcome to Nithin's Robot interface!                  *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*       [1] Start RosAria on Robot                               *" << std::endl
			<< "*       [2] get_master_ip                                        *" << std::endl
			<< "*       [3] get_system_readings                                  *" << std::endl
			<< "*       [4] get_master_port                                      *" << std::endl
			<< "*       [5] ---------------------------                          *" << std::endl
			<< "*       [6] -------------                                        *" << std::endl
			<< "*       Press [Q] to close the interface                         *" << std::endl 
			<< "******************************************************************" << std::endl;

	char select,a,b; /* vars to be used in switch statement */

  /* loop to handle user input */
	while(ros::ok()){


		std::cout << "Please select a program to run, or hit q to quit: "<< std::endl; /* prompt user at start of every loop */
		std::cin >> select;	/* use standard input to select program to run */


		switch(select)
		{
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
			case KEYCODE_1:
			{
				if (ros::master::check())
				{
				  //a=system("export ROS_MASTER_URI=%s", ros::master::getURI());
					a=system("rosrun robot_interface start_rosaria");
				}
			}
				break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			case KEYCODE_2:
			{
				//const std::string&(master_ip)(ros::master::getHost);
				std::cout<<ros::master::getURI()<<std::endl;
			}
				break;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
			case KEYCODE_Q: /* quit the interface program */
        quit(); /* reset the terminal, then quit + exit the program */
        return false;
				break;
			default: /* in case of user not entering selection from list */
				std::cout << "Please enter a number from the list or Q to quit." << std::endl;
				break;
		}
	}
	return 1;
}
