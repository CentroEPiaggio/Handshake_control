// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>

#include <fstream> 
#include "ros/ros.h"
#include <ros/node_handle.h>
#include <stdio.h>
#include <iostream>

#include <stdlib.h>
#include <ros/console.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Int32.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <qb_interface/adcSensorArray.h>
#include <qb_interface/handPos.h>
#include <qb_interface/handRef.h>
#include <qb_interface/handCurrent.h>
#include <handshake_ekf/StateVec.h>

#include <handshake_topic_synchronizer/PoseStamped.h>
#include <handshake_topic_synchronizer/Float64MultiArrayStamped.h>
#include <handshake_topic_synchronizer/handPosStamped.h>
#include <handshake_topic_synchronizer/Int16Stamped.h>
#include <handshake_topic_synchronizer/handCurrentStamped.h>
#include <handshake_topic_synchronizer/handRefStamped.h>
#include <handshake_topic_synchronizer/stateVecStamped.h>
#include <handshake_topic_synchronizer/adcSensorArrayStamped.h>

#include <ros/ros.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
   	pub_st_equilibrium_pose         	= node.advertise<handshake_topic_synchronizer::PoseStamped>("/st_panda_arm_equilibrium_pose",1);
  	pub_st_desired_stiffness_matrix		= node.advertise<handshake_topic_synchronizer::Float64MultiArrayStamped>("/st_panda_arm_desired_stiffness_matrix",1);
  	pub_st_feedback_flag             	= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_feedback_flag_activation",1);
  	pub_st_hand_measurement          	= node.advertise<handshake_topic_synchronizer::handPosStamped>("/st_qb_class_handPos",1);
  	pub_st_hand_current           		= node.advertise<handshake_topic_synchronizer::handCurrentStamped>("/st_qb_class_handCurrent",1);
  	pub_st_hand_reference           	= node.advertise<handshake_topic_synchronizer::handRefStamped>("/st_qb_class_handRef",1);
  	pub_st_pressure_flag  				= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_pressure_feedback_activation",1);
  	pub_st_arm_stiffness      			= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_current_arm_stiffness",1);
  	pub_st_ekf_pose  		 			= node.advertise<handshake_topic_synchronizer::PoseStamped>("/st_handshake_EKF_controlled_pose",1);
  	pub_st_ekf_state  		 			= node.advertise<handshake_topic_synchronizer::stateVecStamped>("/st_handshake_EKF_state",1);
  	pub_st_qb_adc    					= node.advertise<handshake_topic_synchronizer::adcSensorArrayStamped>("/st_qb_class_imu_adc",1);


    //Topic you want to subscribe
    sub_equilibrium_pose         	= node.subscribe("/panda_arm/equilibrium_pose",1, &SubscribeAndPublish::pose_callback, this);
   	sub_desired_stiffness_matrix	= node.subscribe("/panda_arm/desired_stiffness_matrix",1, &SubscribeAndPublish::K_callback, this);
   	sub_feedback_flag             	= node.subscribe("handshake_feedback_flag_activation",1, &SubscribeAndPublish::f_flag_callback, this);
   	sub_hand_measurement          	= node.subscribe("/qb_class/handPos",1, &SubscribeAndPublish::q_callback, this);
   	sub_hand_current            	= node.subscribe("/qb_class/handCurrent",1, &SubscribeAndPublish::I_callback, this);
   	sub_hand_ref            		= node.subscribe("/qb_class/handRef",1, &SubscribeAndPublish::ref_callback, this);
   	sub_pressure_flag  				= node.subscribe("handshake_pressure_feedback_activation",1, &SubscribeAndPublish::p_flag_callback, this);
   	sub_arm_stiffness      			= node.subscribe("handshake_current_arm_stiffness",1, &SubscribeAndPublish::k_callback, this);
   	sub_ekf_pose   		 			= node.subscribe("/handshake_EKF_controlled_pose",1, &SubscribeAndPublish::ekf_pose_callback, this);
   	sub_ekf_state   		 		= node.subscribe("/handshake_EKF_state",1, &SubscribeAndPublish::ekf_state_callback, this);   	
    //sub_qb_adc              = node.subscribe("/qb_class_imu/adc",1, &SubscribeAndPublish::adc_callback, this);
    sub_qb_adc    					= node.subscribe("handshake_post_adc_sensors",1, &SubscribeAndPublish::adc_callback, this);
  }

  // void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  // {
  //   PUBLISHED_MESSAGE_TYPE output;
  //   //.... do something with the input and generate the output...
  //   pub_.publish(output);
  // }

  void pose_callback(const geometry_msgs::Pose& pose_msg)
  {
    handshake_topic_synchronizer::PoseStamped st_pose_msg;

    st_pose_msg.header.stamp 	= ros::Time::now();
    st_pose_msg.position 			= pose_msg.position;
    st_pose_msg.orientation 		= pose_msg.orientation;

    pub_st_equilibrium_pose.publish(st_pose_msg);
  }

  void K_callback(const std_msgs::Float64MultiArray& K_msg)
  {
    handshake_topic_synchronizer::Float64MultiArrayStamped st_K_msg;

    st_K_msg.header.stamp 	= ros::Time::now();
    st_K_msg.layout 		= K_msg.layout;
	st_K_msg.data 			= K_msg.data;

    pub_st_desired_stiffness_matrix.publish(st_K_msg);
  }

  void f_flag_callback(const std_msgs::Int16& f_flag_msg)
  {
    handshake_topic_synchronizer::Int16Stamped st_f_flag_msg;

    st_f_flag_msg.header.stamp 	= ros::Time::now();
	st_f_flag_msg.data 			= f_flag_msg.data;

    pub_st_feedback_flag.publish(st_f_flag_msg);
  }

  void p_flag_callback(const std_msgs::Int16& p_flag_msg)
  {
    handshake_topic_synchronizer::Int16Stamped st_p_flag_msg;

    st_p_flag_msg.header.stamp 	= ros::Time::now();
	st_p_flag_msg.data 			= p_flag_msg.data;

    pub_st_pressure_flag.publish(st_p_flag_msg);
  }

  void q_callback(const qb_interface::handPos& q_msg)
  {
    handshake_topic_synchronizer::handPosStamped st_q_msg;

    st_q_msg.header.stamp 	= ros::Time::now();
    st_q_msg.closure		= q_msg.closure;

    pub_st_hand_measurement.publish(st_q_msg);
  }

  void I_callback(const qb_interface::handCurrent& I_msg)
  {
    handshake_topic_synchronizer::handCurrentStamped st_I_msg;

    st_I_msg.header.stamp 	= ros::Time::now();
    st_I_msg.current		= I_msg.current;

    pub_st_hand_current.publish(st_I_msg);
  }

  void ref_callback(const qb_interface::handRef& ref_msg)
  {
    handshake_topic_synchronizer::handRefStamped st_ref_msg;

    st_ref_msg.header.stamp 	= ros::Time::now();
    st_ref_msg.closure			= ref_msg.closure;

    pub_st_hand_reference.publish(st_ref_msg);
  }

  void k_callback(const std_msgs::Int16& k_msg)
  {
    handshake_topic_synchronizer::Int16Stamped st_k_msg;

    st_k_msg.header.stamp 	= ros::Time::now();
	st_k_msg.data 			= k_msg.data;

    pub_st_arm_stiffness.publish(st_k_msg);
  }

   void ekf_pose_callback(const geometry_msgs::Pose& ekf_p_msg)
  {
    handshake_topic_synchronizer::PoseStamped st_ekf_p_msg;

    st_ekf_p_msg.header.stamp 	= ros::Time::now();
	st_ekf_p_msg.position 		= ekf_p_msg.position;
	st_ekf_p_msg.orientation 	= ekf_p_msg.orientation;

    pub_st_ekf_pose.publish(st_ekf_p_msg);
  }

  void ekf_state_callback(const handshake_ekf::StateVec& ekf_s_msg)
  {
    handshake_topic_synchronizer::stateVecStamped st_ekf_s_msg;

    st_ekf_s_msg.header.stamp 	= ros::Time::now();
	st_ekf_s_msg.x1 			= ekf_s_msg.x1;
	st_ekf_s_msg.x2 			= ekf_s_msg.x2;
	st_ekf_s_msg.x3 			= ekf_s_msg.x3;
	st_ekf_s_msg.x4 			= ekf_s_msg.x4;
	st_ekf_s_msg.z 				= ekf_s_msg.z;
	st_ekf_s_msg.zd 			= ekf_s_msg.zd;

    pub_st_ekf_state.publish(st_ekf_s_msg);
  }

  void adc_callback(const qb_interface::adcSensorArray& adc_msg)
  {
    handshake_topic_synchronizer::adcSensorArrayStamped st_adc_msg;

    st_adc_msg.header.stamp 		= ros::Time::now();
	st_adc_msg.m[0].adc_sensor_1	= adc_msg.m[0].adc_sensor_1;
	st_adc_msg.m[0].adc_sensor_2	= adc_msg.m[0].adc_sensor_2;

    pub_st_qb_adc.publish(st_adc_msg);
  }


private:
  ros::NodeHandle node; 

  ros::Publisher pub_st_equilibrium_pose;       
  ros::Publisher pub_st_desired_stiffness_matrix;	
  ros::Publisher pub_st_feedback_flag;             
  ros::Publisher pub_st_hand_measurement; 
  ros::Publisher pub_st_hand_current;           
  ros::Publisher pub_st_hand_reference;  
  ros::Publisher pub_st_pressure_flag; 				
  ros::Publisher pub_st_arm_stiffness;      			
  ros::Publisher pub_st_ekf_pose;  		 			
  ros::Publisher pub_st_ekf_state; 
  ros::Publisher pub_st_qb_adc;    		

  ros::Subscriber sub_equilibrium_pose;         	
  ros::Subscriber sub_desired_stiffness_matrix;	
  ros::Subscriber sub_feedback_flag;             	
  ros::Subscriber sub_hand_measurement;          	
  ros::Subscriber sub_hand_current;            	 
  ros::Subscriber sub_hand_ref;            	
  ros::Subscriber sub_pressure_flag;  				
  ros::Subscriber sub_arm_stiffness;      			
  ros::Subscriber sub_ekf_pose;   		 			
  ros::Subscriber sub_ekf_state;   		 		
  ros::Subscriber sub_qb_adc; 					

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "handshake_topic_synchronizer_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  //ros::spin();
  ros::Rate rate(100);

  while (ros::ok()){
      //Spinning once to get messages from topics
    ros::spinOnce();
    rate.sleep();

  }

  return 0;
}

// using namespace sensor_msgs;
// using namespace message_filters;

// void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
// {
//   // Solve all of perception here...
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "handshake_topic_synchronizer");

//   ros::NodeHandle nh;

//   ros::Publisher pub_st_equilibrium_pose         	= node.advertise<handshake_topic_synchronizer::PoseStamped>("/st_panda_arm_equilibrium_pose",1);
//   ros::Publisher pub_st_desired_stiffness_matrix	= node.advertise<handshake_topic_synchronizer::Float64MultiArrayStamped>("/st_panda_arm_desired_stiffness_matrix",1);
//   ros::Publisher pub_st_feedback_flag             	= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_feedback_flag_activation",1);
//   ros::Publisher pub_st_hand_measurement          	= node.advertise<handshake_topic_synchronizer::handPosStamped>("/st_qb_class_handPos",1);
//   ros::Publisher pub_st_hand_current           		= node.advertise<handshake_topic_synchronizer::handCurrentStamped>("/st_qb_class_handCurrent",1);
//   ros::Publisher pub_st_pressure_flag  				= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_pressure_feedback_activation",1);
//   ros::Publisher pub_st_arm_stiffness      			= node.advertise<handshake_topic_synchronizer::Int16Stamped>("/st_handshake_current_arm_stiffness",1);
//   ros::Publisher pub_st_ekf_pose  		 			= node.advertise<handshake_topic_synchronizer::PoseStamped>("/st_handshake_EKF_controlled_pose",1);
//   ros::Publisher pub_st_ekf_state  		 			= node.advertise<handshake_topic_synchronizer::stateVecStamped>("/st_handshake_EKF_state",1);
//   ros::Publisher pub_st_qb_adc    					= node.advertise<handshake_topic_synchronizer::adcSensorArrayStamped>("/st_qb_class_imu_adc",1);

//   ros::Subscriber sub_equilibrium_pose         	= node.subscribe("/panda_arm/equilibrium_pose",1);
//   ros::Subscriber sub_desired_stiffness_matrix	= node.subscribe("/panda_arm/desired_stiffness_matrix",1);
//   ros::Subscriber sub_feedback_flag             = node.subscribe("handshake_feedback_flag_activation",1);
//   ros::Subscriber sub_hand_measurement          = node.subscribe("/qb_class/handPos",1);
//   ros::Subscriber sub_hand_current            	= node.subscribe("/qb_class/handCurrent",1);
//   ros::Subscriber sub_pressure_flag  			= node.subscribe("handshake_pressure_feedback_activation",1);
//   ros::Subscriber sub_arm_stiffness      		= node.subscribe("handshake_current_arm_stiffness",1);
//   ros::Subscriber sub_ekf_pose   		 		= node.subscribe("/handshake_EKF_controlled_pose",1);
//   ros::Subscriber sub_ekf_state   		 		= node.subscribe("/handshake_EKF_state",1);
//   ros::Subscriber sub_qb_adc    				= node.subscribe("/qb_class_imu/adc",1);


//   handshake_topic_synchronizer::stateVecStamped state_msg;
//   // message_filters::Subscriber<Image> image_sub(nh, "image", 1);
//   // message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
//   // TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
//   // sync.registerCallback(boost::bind(&callback, _1, _2));

//   //ros::spin();

//   while (ros::ok()){
//     //Spinning once to get messages from topics
//     ros::spinOnce();

//     state_msg.header.stamp 	= ros::Time::now();
//     state_msg.x1			= sub_ekf_state; 
//     state_msg.x2 			= X(1); 
//     state_msg.x3 			= X(2); 
//     state_msg.x4 			= X(3); 
//     state_msg.z 			= X(4); 
//     state_msg.zd 			= X(5); 
//     pub_st_ekf_state .publish(state_msg);



//     ros::spinOnce();
//     rate.sleep();

//   }

//   //return 0;
// }