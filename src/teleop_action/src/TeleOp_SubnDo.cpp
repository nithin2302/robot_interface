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
    action_name_(name),
    linear_(0),
    angular_(0),
    l_scale_(2.0),
    a_scale_(2.0)
  {
    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/key_press", 1, &TeleOpAction::movementCB, this);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  }

  ~TeleOpAction(void)
  {
  }


  void movementCB(const std_msgs::Int32::ConstPtr& msg)
  {

    bool quit_request = false;
    bool dirty = false;
    //for(;;)
    {
        int keyread = msg->data;
        //Based on Keypress, send movement data to AGV
        ROS_INFO("Robot Moving in the direction : ");
        //if (keyread!=0)
        switch(msg->data)
          {
            case 1:
              ROS_INFO("LEFT \n");
              angular_ = 0.1;
              linear_ = 0;
              dirty = true;
              break;
            case 2:
              ROS_INFO("RIGHT \n");
              angular_ = -0.1;
              linear_ = 0;
              dirty = true;
              break;
            case 3:
              ROS_INFO("UP \n");
              linear_ = 0.1;
              angular_ = 0;
              dirty = true;
              break;
            case 4:
              ROS_INFO("DOWN \n");
              linear_ = -0.1;
              angular_ = 0;
              dirty = true;
              break;
            case 5:
              ROS_INFO("STOP \n");
              linear_ = 0;
              angular_ = 0;
              dirty = true;
              break;
            case 6:
              linear_ = 0;
              angular_ = 0;
              dirty = true;
              ROS_DEBUG("QUIT \n");
              ROS_INFO_STREAM("You quit the teleop successfully");
              quit_request = true;
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
          ROS_INFO("Cycle : %i", keyread);
          keyread=0;
          dirty=false;
        }

        if(quit_request==true)
        {
          return;
          //break;
        }
    }
  }

protected:

  ros::NodeHandle nh_;
  std::string action_name_;
  int goal_;
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
