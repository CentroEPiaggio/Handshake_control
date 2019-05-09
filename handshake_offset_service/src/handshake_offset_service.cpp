#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Int32.h>
#include "geometry_msgs/Twist.h"
#include <fstream> 
#include "ros/ros.h"
#include <ros/node_handle.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <ros/console.h>
#include <tf/transform_listener.h>


using namespace std;

/*---------------------------------------------------------------------*
* GLOBAL VARIABLE DEFINITIONS                                          *
*                                                                      *
*----------------------------------------------------------------------*/

trajectory_msgs::JointTrajectory hand_offset_cl_msg;
double hand_cl_max    = 19000.0;
double hand_offset_cl = 0.0;

/*---------------------------------------------------------------------*
* GET CHAR FROM KEYBOARD                                               *
*                                                                      *
*----------------------------------------------------------------------*/

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

/*---------------------------------------------------------------------*
* MAIN                                                                 *
*                                                                      *
*----------------------------------------------------------------------*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "handshake_offset_service_node");

  ros::NodeHandle node;

  int digit;
  char term;

  ros::Rate rate(100);

  ros::Publisher pub_hand_offset_cl = node.advertise<trajectory_msgs::JointTrajectory>("/right_hand/joint_trajectory_controller/command",1);
         
  while (ros::ok()){
    //Spinning once to get messages from topics
    ros::spinOnce();

    for(int i=0; i=100; i++){

      double actual_cl = i/100.0;
    
    // Creating traj msg for hand controller
      hand_cl_msg.points.clear();
      hand_cl_msg.joint_names.clear();
      hand_cl_msg.joint_names.push_back("right_hand_synergy_joint");

      trajectory_msgs::JointTrajectoryPoint start_point;
      start_point.time_from_start = ros::Duration(1.0/50.0);

      start_point.positions.clear();
      start_point.positions.push_back((hand_cl_max*actual_cl);

      hand_cl_msg.points.push_back(start_point);

      pub_hand_cl.publish(hand_offset_cl_msg);

      hand_offset_cl = hand_cl_max*actual_cl;

      if (kbhit()){
      (void)getch();

      std::cout << "Set hand offset angle to " << hand_offset_cl << std::endl;
      return 0;
      }

      ros::spinOnce();
      rate.sleep();

    }

  }
  
}