////////////////////////////////////// Nithin ///////////////////////////////////////////////////////


/* This is an ArNetworking client built to be used with the Pb API.
 * It connects to an ArNetworking server and makes the robot go to the given goal
 * Server is run by just running this function. It must be run after running: 
 * sudo /usr/local/Arnl/examples/hv/pbapiServer -map /var/www/1scans.map where 1scans.map is the map you wish to have on the server
 * The function integrates the previously isolated GoTo functions from the PB API module. All goal values i.e. goal, home and dock can be called to this server.
 */

////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Aria.h"
#include "ArNetworking.h"

#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <sstream>

#include "ros/ros.h"
#include "ros/master.h"
#include <std_msgs/String.h>
#include "robot_interface/PBGo.h"

bool done;
using namespace std;

class InputHandler
{
public:
  InputHandler(ArClientBase *client, char *goal);
  virtual ~InputHandler(void);

  void go();
  void dock();
  void home();

protected:
  ArClientBase *myClient;
  char *myGoal;

  ///@{
  ArFunctorC<InputHandler> myGoCB;
  ArFunctorC<InputHandler> myDockCB;
  ArFunctorC<InputHandler> myHomeCB;
  ///@}
};

InputHandler::InputHandler(ArClientBase *client, char *goal) :
  myClient(client), myGoal(goal),myDockCB(this, &InputHandler::dock),myHomeCB(this, &InputHandler::home), myGoCB(this, &InputHandler::go)

  /* Initialize functor objects with pointers to our handler methods: */
 
{
}

InputHandler::~InputHandler(void)
{

}

void InputHandler::go()
{
  ArNetPacket packet;
  packet.strToBuf(myGoal);
  myClient->requestOnce("gotoGoal", &packet);
}

void InputHandler::dock()
{
  myClient->requestOnce("dock");
}

void InputHandler::home()
{
  myClient->requestOnce("home");
}

class OutputHandler
{
public:
  OutputHandler();
  OutputHandler(ArClientBase *client);
  OutputHandler(ArClientBase *client, char *goal);
  virtual ~OutputHandler(void);

  void update(ArNetPacket *packet);
  char myStatus[256];
  char myMode[256];

  int exitCode;
  char *myGoal;
  string goalstr;

protected:
  ArClientBase *myClient;
  int first;

  ///@{
  ArFunctor1C<OutputHandler, ArNetPacket *> myUpdateCB;
  ///@}
};

OutputHandler::OutputHandler(ArClientBase *client, char *goal) :
  exitCode(-1), myClient(client), first(0), myGoal(goal),

  /* Initialize functor objects with pointers to our handler methods: */
  myUpdateCB(this, &OutputHandler::update)
{
  myClient->addHandler("updateStrings", &myUpdateCB);
  myClient->request("updateStrings", -1);
}

OutputHandler::~OutputHandler(void)
{

}

void OutputHandler::update(ArNetPacket *packet)
{
  memset(myStatus, 0, sizeof(myStatus));
  memset(myMode, 0, sizeof(myMode));
  packet->bufToStr(myStatus, sizeof(myStatus));
  packet->bufToStr(myMode, sizeof(myMode));

  cout << "Status: " << myStatus << endl;
  cout << "Mode: " << myMode << endl;
  cout << "exitCode : " << exitCode << endl;
  //Debug
  //cout << goal << endl;

  //This is some of the strings that the robot server outputs when it does something or fails to do something.
  string temp = "Arrived at " + (string)myGoal;
  //Debug
  //cout << "temp is: " << temp << endl;

  string temp1 = "Failed to get to ";
  string temp2 = " (Failed going to goal)";
  temp1 = temp1 + myGoal + temp2;

  string temp3 = "Failed to get to ";
  string temp4 = " (Robot lost)";
  temp3 = temp3 + myGoal + temp4;

  string  temp5 = "Failed to get to ";
  string temp6 = " (Cannot find path)";
  temp5 = temp5 + myGoal + temp6;

  //Debug
  //cout <<  temp << endl;
  //cout << myStatus << endl;

//  if(myStatus == temp || !strcmp(myStatus, "Returned home") || !strcmp(myStatus, "Docked") || !strcmp(myStatus, "Arrived at point"))
  if(myStatus == temp)
  {
   done=true;
   exitCode = 0;
  }
  else if(myStatus == temp1 || myStatus == temp3 || myStatus == temp5)
  {
	done=true;    
	exitCode = 3;
  }
}

bool GoTo(robot_interface::PBGo::Request  &req,
         robot_interface::PBGo::Response &res)
{
		/* Aria initialization: */
  Aria::init();
  done=false;

  //ArLog::init(ArLog::StdErr, ArLog::Verbose);

  /* Create our client object. This is the object which connects with a remote
   * server over the network, and which manages all of our communication with it
   * once connected by sending data "requests".  Requests may be sent once, or
   * may be repeated at any frequency. Requests and replies to requsets contain
   * payload "packets", into which various data types may be packed (when making a
   * request), and from which they may also be extracted (when handling a reply).
   * See the InputHandler and OutputHandler classes above for
   * examples of making requests and reading/writing the data in packets.
   */
  ArClientBase client;
  ArArgumentBuilder *args = new ArArgumentBuilder();
  /* Aria components use this to get options off the command line: */
  ArArgumentParser *parser = new ArArgumentParser(args);
  args->add("-robotPort"); // pass robot's serial port to Aria
  args->add("/dev/ttyS0");
  //ArArgumentParser parser(&argc, argv);
  parser->loadDefaultArguments();
  /* This will be used to connect our client to the server, including
   * various bits of handshaking (e.g. sending a password, retrieving a list
   * of data requests and commands...)
   * It will get the hostname from the -host command line argument: */
  ArClientSimpleConnector clientConnector(args);

  

  /* Check for -help, and unhandled arguments: */
  if (!Aria::parseArgs() || !parser->checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    exit(2);
  }

  /* Connect our client object to the remote server: */
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      printf("Server '%s' rejected connection, exiting\n", client.getHost());
    else
      printf("Could not connect to server '%s', exiting\n", client.getHost());
    exit(2);
  }

  printf("Connected to server.\n");

  client.setRobotName(client.getHost()); // include server name in log messages

  /* Now that we're connected, we can run the client in a background thread,
   * sending requests and receiving replies. When a reply to a request arrives,
   * or the server makes a request of the client, a handler functor is invoked.
   * The handlers for this program are registered with the client by the
   * InputHandler and OutputHandler classes (in their constructors, above) */
    client.runAsync();
	char *request = (char*)"";
    //robot_interface::PBGo::Request  &req;
    //robot_interface::PBGo::Response &res

  if (client.getRunningWithLock())
  {
		//const char* request;
		request = strdup(req.goal_name.c_str());
		//request = req.goal_name.c_str();
    if (req.goal_name=="dock"|| req.goal_name=="DOCK" || req.goal_name=="Dock")
    {
        InputHandler inputHandler(&client, "dock");
        inputHandler.dock();
        std::cout<<"Moving to Dock Position"<<endl;
    }
    else if (req.goal_name=="home"|| req.goal_name=="HOME" || req.goal_name=="Home")
    {
        InputHandler inputHandler(&client, "home");
        inputHandler.home();
        std::cout<<"Moving to Home Position"<<endl;
    }


    else if(std::strstr (req.goal_name.c_str(),"goal"))
    {
      if(1)
      {
        InputHandler inputHandler(&client, request);
        inputHandler.go();
	cout << "Goal is: " << request << endl;    
      }

    }
    else
    {
      cout << "No valid arg" << endl;
      exit(2);
    }

    OutputHandler outputHandler(&client, request);

    //outputHandler.goal = goal;

    if(strcmp(request, ""))     //true when request != ""
    {
      outputHandler.myGoal = request;
    }
    else
      outputHandler.myGoal = (char*)"";

    do
	{
      //if (done)
        if(outputHandler.exitCode == 0)
		{
		  res.PB_reached=true;
		  client.disconnect();
	  	  Aria::shutdown();
	 	  //return 0;
		  break;
		}

	}
	while(!done);
    return (1);
  }
	else
  /* The client stopped running, due to disconnection from the server, general
   * Aria shutdown, or some other reason. */
  //client.disconnect();
  //Aria::shutdown();
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PBGo_server");
  ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("PBGo", GoTo);
	ROS_INFO("Waiting for goal to move... \n");
	ros::spin();
  return 0;

}
