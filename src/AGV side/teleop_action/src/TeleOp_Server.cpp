////////////////////////////////////// Nithin ///////////////////////////////////////////////////////


/* ActionServer used to TeleOp the AGV
 * Run on the AGV
 * Reads keypress values from the topic "/key_press" and moves AGV accordingly 
 * Moves AGV only after TeleOp_Client is run
 */

////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <teleop_action/TeleOpAction.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

class TeleOpAction
{
public:

  TeleOpAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    linear_(0),
    angular_(0),
    l_scale_(2.0),
    a_scale_(2.0)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&TeleOpAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&TeleOpAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/key_press", 1, &TeleOpAction::movementCB, this);
    as_.start();

    {
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    }
  }

  ~TeleOpAction(void)
  {
  }

  void goalCB()
  {

    // accept the new goal
    goal_ = as_.acceptNewGoal()->samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void movementCB(const std_msgs::Int32::ConstPtr& msg)
  {
	
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    bool quit_request=true;
    bool dirty = false;
    feedback_.Quit = quit_request;

    //Based on Keypress, send movement data to AGV
    ROS_INFO("Robot Moving in the direction : ");
    //if (keyread!=0)
    switch(msg->data)
	{        
		
	case 1:
          ROS_INFO("LEFT");
          angular_ = 0.1;
          linear_ = 0;
          dirty = true;
	  quit_request=false;
          break;
        case 2:
          ROS_INFO("RIGHT");
          angular_ = -0.1;
          linear_ = 0;
          dirty = true;
	quit_request=false;
          break;
        case 3:
          ROS_INFO("UP");
          linear_ = 0.1;
          angular_ = 0;
          dirty = true;
	quit_request=false;
          break;
        case 4:
          ROS_INFO("DOWN");
          linear_ = -0.1;
          angular_ = 0;
          dirty = true;
	quit_request=false;
          break;
        case 5:
          ROS_INFO("STOP");
          linear_ = 0;
          angular_ = 0;
          dirty = true;
	quit_request=false;
	result_.Quit=false;
          break;
        case 6:
          linear_ = 0;
          angular_ = 0;
          dirty = true;
          //ROS_DEBUG("QUIT");
          //ROS_INFO("You quit the teleop successfully");
          quit_request = true;
          ROS_INFO("Quitting");
          result_.Quit=true;
          as_.setSucceeded(result_);
          return;
          break;

        default:
            ROS_INFO("Please Use the Arrow keys to move the Robot, SPACE to stop and Q to quit");

	}
        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_*angular_;
        twist.linear.x = l_scale_*linear_;
        if(dirty==true)
        {
          twist_pub_.publish(twist);
          ROS_INFO("Cycle : ");
          dirty=false;
        }
        as_.publishFeedback(feedback_);  
	return;    
  }

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<teleop_action::TeleOpAction> as_;
  std::string action_name_;
  int goal_;
  teleop_action::TeleOpFeedback feedback_;
  teleop_action::TeleOpResult result_;
  ros::Subscriber sub_;
  double linear_, angular_, l_scale_, a_scale_;
  
  ros::Publisher twist_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Teleop");

  TeleOpAction teleop(ros::this_node::getName());
  ros::spin();

  return 0;
}
