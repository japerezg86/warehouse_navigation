#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "point_c_navigation_goal");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Time begin;
  ros::Time end;
  ros::Duration duration;

  //define the move base goals for each room
  move_base_msgs::MoveBaseGoal point_c_goal;

  //send the second goal
  point_c_goal.target_pose.header.frame_id = "map";
  point_c_goal.target_pose.header.stamp = ros::Time::now();

  point_c_goal.target_pose.pose.position.x = -14.6357506747;
  point_c_goal.target_pose.pose.position.y = 3.49857788575;
  point_c_goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending point_c_goal");

  begin = ros::Time::now();
  ac.sendGoal(point_c_goal);

  ac.waitForResult();
  end = ros::Time::now();

  duration = end - begin;
  cout << "Time to reach goal in sec: " << duration.toSec() << endl;

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the task has been achieved");
  else
    ROS_INFO("The base failed to achieve task for some reason");

  return 0;
}