#include <iostream>
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#define TOL 0.5

geometry_msgs::Pose current;
geometry_msgs::Pose initial;
bool flag = true;

void odomCB(const nav_msgs::Odometry& msg)
{
  if(flag == true)
  {
    flag = false;
    initial = msg.pose.pose;
  }
  else
  {
    current = msg.pose.pose;
  }
  //std::cout << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << std::endl;
}

/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "SwarmSimu");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  ros::Subscriber odom_data = n.subscribe("/swarmbot0/odom",10,odomCB);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Rate loop_rate(5);

  int count = 0;

  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = 0.4;

  bool flipped = false;

  while (ros::ok())
  {
    nav_msgs::Odometry odom;
    cmd_vel.linear.x = 1;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    // cmd_vel.angular.z = 0.4*sin(count);

    if((current.position.x < initial.position.x + TOL && current.position.x > initial.position.x - TOL)&&(current.position.y < initial.position.y + TOL && current.position.y > initial.position.y - TOL))
      {
        if(flipped==false)
        {
          cmd_vel.angular.z = -cmd_vel.angular.z;
          flipped = true;
          initial = current;
        }
      }
    else
      flipped = false;
    vel_pub_0.publish(cmd_vel);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}
