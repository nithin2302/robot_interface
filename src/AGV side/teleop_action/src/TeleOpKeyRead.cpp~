#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class TeleOpKeyRead
{
  public:
    TeleOpKeyRead();
    void keyLoop();
  private:
    ros::NodeHandle nh_;
    ros::Publisher key_press_pub;
};
TeleOpKeyRead::TeleOpKeyRead()

{
  key_press_pub = nh_.advertise<std_msgs::Int32>("/key_press", 1);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_KeyRead");
  TeleOpKeyRead teleop_KeyRead;
  signal(SIGINT,quit);
  teleop_KeyRead.keyLoop();
  return(0);
}
void TeleOpKeyRead::keyLoop()
{
  char c;
  bool dirty=false;
  int32_t keycode=0;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }

    std_msgs::Int32 msg;
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
    	  dirty = true;
    	  keycode=1;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
    	  dirty = true;
    	  keycode=2;
    	  break;
    	case KEYCODE_U:
    	  ROS_DEBUG("UP");
    	  dirty = true;
    	  keycode=3;
    	  break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
    	  dirty = true;
    	  keycode=4;
    	  break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
    	  dirty = true;
    	  keycode=5;
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        keycode=6;
	dirty=true;
        break;
  	}
    msg.data=keycode;
    
    if(dirty ==true && !keycode==0)
  	{ 
	  key_press_pub.publish(msg);	  
  	  dirty=false;
  	  keycode=0;
  	}
    if (keycode == 6)
	{
	  ROS_INFO_STREAM("Quitting TeleOpKeyRead");
	  return;
	  break;
	}
  }
  return;
}
