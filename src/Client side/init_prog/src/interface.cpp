////////////////////////////////////// Nithin ///////////////////////////////////////////////////////


/* Simple interface that can be used for any number of function calls from the CLI
 */

////////////////////////////////////////////////////////////////////////////////////////////////////

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

	if (ros::master::check){
		std::string x;
		//x=system("ifconfig wlan0|grep 'inet addr:' | cut -d: -f2 | awk '{print $1}'");
        x=system("hostname -I");
		std::cout<<"Master is running on network on the id : " ;
		std::cout<< ros::master::getURI()<<std::endl;
		}

  /* change the terminal input settings */
	initTermios(0);

  /* greet user and display selection options */
	std::cout
			<< "******************************************************************" << std::endl
			<< "*                   NITHIN'S ROS INTERFACE                       *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*            Welcome to Nithin's ROS interface!                  *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*       [1] make_current_system_master                           *" << std::endl
			<< "*       [2] get_master_ip                                        *" << std::endl
			<< "*       [3] List Nodes running on Robot                          *" << std::endl
			<< "*       [4] Get output of nodes on rqt_gui                       *" << std::endl
			<< "*       [5] ##                 Start Mapping                ##   *" << std::endl
			<< "*       [6] ##              Navigation Options              ##   *" << std::endl
			<< "*       Press [Q] to close the interface                         *" << std::endl
			<< "******************************************************************" << std::endl;

	char select,a,b; /* vars to be used in switch statement */

  /* loop to handle user input */
	while(ros::ok()){
		std::cout << "Please select a program to run, or hit q to quit: "<< std::endl; /* prompt user at start of every loop */
		std::cin >> select;	/* use standard input to select program to run */


		switch(select)
		{

			case KEYCODE_1:
			{
				if (ros::master::check())
				{
					std::cout<<"roscore is already running."<<std::endl;/* run option 1*/
					//a = system("rosnode kill /roscore"); /*kill the roscore node */
					//a = system("rosrun init_prog start_roscore");//Start Roscore on the current terminal to make it master
				}
				else a = system("gnome-terminal -e 'sh \"rosrun init_prog start_roscore\"'");
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

			case KEYCODE_3:
			{
				std::cout<< "The Following Nodes are running on the Patrolbot : "<<std::endl;
				a = system("rostopic list");
			}

				break;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			case KEYCODE_4:
			{
				a = system("gnome-terminal -x sh -c \"rosrun rqt_gui rqt_gui\"");
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
