Author : Nithin Ramesh

Contact : nithin2302@gmail.com



The project consists of the following functionalities :



* TeleOperation

* Mapping

* Transfer of Maps

* Navigation



INITIAL STEPS :



1. Copy all folders in the AGV side folder to the src folder of the catkin workspace on the AGV


2. Copy all folders in the Client side folder to the src folder of the catkin workspace on the client console


3. $ cd catkin_ws (on both AGV and Client consoles)


4. $ catkin_make (on both AGV and Client consoles)





*************************    TELEOPERATION   ********************************

-----------------------------------------------------------------------------
OPTION 1:
-----------------------------------------------------------------------------


On AGV :

$ rosrun teleop_action TeleOp_Server


On Client console :

$ rosrun teleop_action TeleOpKeyRead

$ rosrun teleop_action TeleOp_Client


Use the keyboard to jog the AGV around.


-------------------------------------------------------------------------------
OPTION 2 : 
-------------------------------------------------------------------------------


On AGV :

$ rosrun teleop_action TeleOp_SubnDo


On Client Console :

$ rosrun teleop_action TeleOpKeyRead

Use Keyboard to jog the AGV around




********************************************************************************

******

**********************   MAPPING & TRANSFER OF MAP   *********************


On AGV :

$ rosrun robot_interface ROS_sickLogger


Wait for confirmation of lasers connected, then jog the robot around the area for mapping


On Client Side :

$ rosrun init_prog get_map


Once running, you can end mapping on AGV and press esc through puTTY


Map is transferred to home/receivedMap.2d


*********************************************************************************

******

***************************   NAVIGATION   ********************************



On AGV :

$ sudo /usr/local/Arnl/examples/hv/pbapiServer -map /var/www/1scans.map


where, 1scans.map is the name of the map to be used

Once this is running,

$ rosrun robot_interface PBGo_server


On Client console :

$ rosrun init_prog PBGo_client


NOTE : Communication between client and server curently not happening. May be due to different versions of OS on the two systems. 
       Needs to be fixed. 
Currently, the same command can be run on the AGV using puTTY.

Once client prompts user to enter a goal, goal can be entered as named in the map.


***************************************************************************************








