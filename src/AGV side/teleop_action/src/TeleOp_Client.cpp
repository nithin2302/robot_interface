////////////////////////////////////// Nithin ///////////////////////////////////////////////////////


/* ActionServer client to be used with TeleOp_Server
 * Sends goal and starts the action. Can be preempted with another client of the same name running on the network
 */

////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <teleop_action/TeleOpAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movement");

  // create the action client
  actionlib::SimpleActionClient<teleop_action::TeleOpAction> ac("Teleop");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  teleop_action::TeleOpGoal goal;
  goal.samples = 10;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
