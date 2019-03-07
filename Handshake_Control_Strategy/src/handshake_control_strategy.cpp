#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
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
#include <qb_interface/adcSensorArray.h>
//#include <qb_interface/handRef.h>
#include <stdlib.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <ros/console.h>


using namespace std;

/*---------------------------------------------------------------------*
* GLOBAL VARIABLE DEFINITIONS                                          *
*                                                                      *
*----------------------------------------------------------------------*/

double pos_hat_x      = 0.6;
double pos_hat_y      = 0;
double pos_hat_z      = 0.5;
double pos_hat_quat_x = 0;
double pos_hat_quat_y = 0;
double pos_hat_quat_z = 0;
double pos_hat_quat_w = 1 ;
double posD_hat       = 0.0;

double k_stiff      = 100.0;
double k_max        = 500.0;
double k_min        = 100.0;
double K_C          = 0.7;
double K_P          = 1.5;
double K_P_stiff    = 2;
double hand_cl      = 0;
double hand_cl_max  = 19000.0;
double hand_max     = 0.75*hand_cl_max;
double cl_th        = 0.2*hand_cl_max;
double max_adc      = 3000.0;

double pressure_sens_1_old  = 0.0;
double pressure_sens_2_old  = 0.0;
double hand_cl_old          = 0.0;
double pres_th              = 100.0;

int flag          = 1;
int control_type  = 1;
int flag_pressure = 0;
ros::Time exp_time;
ros::Time feedback_activation;
ros::Time exp_begin;
ros::Duration pressure_latency(1.5);
ros::Duration exp_finish(30);

trajectory_msgs::JointTrajectory hand_cl_msg;
bool service_called = false;

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
* SATURATION ON VARIABLE CHANGE SMOOTHNESS                             *
*                                                                      *
*----------------------------------------------------------------------*/

double speedSaturation(double actual_value, double previous_value, double threshold)
{

  if (abs(actual_value - previous_value) > threshold) {

    if (previous_value  > actual_value) {
        actual_value = previous_value - threshold/2;
    }

    if (previous_value  < actual_value) {
        actual_value = previous_value + threshold;
    }

  }

  if (abs(actual_value - previous_value) < threshold/5) {

        actual_value = previous_value;

  }

  return actual_value;

}



/*---------------------------------------------------------------------*
* FEEDBACK FOR HANDSHAKE (ARM STIFFNESS AND HAND CLOSURE               *
*                                                                      *
*----------------------------------------------------------------------*/

void qb_adcCallback(const qb_interface::adcSensorArrayConstPtr& pressure_msg){
    
    // Raw values from pressure sensors
    double pressure_sens_1 = pressure_msg->m[0].adc_sensor_1;
    double pressure_sens_2 = pressure_msg->m[0].adc_sensor_2;

    // Saturations on pressure measure smoothness, maxima and minima
    pressure_sens_1 = speedSaturation(pressure_sens_1, pressure_sens_1_old, pres_th);
    pressure_sens_2 = speedSaturation(pressure_sens_2, pressure_sens_2_old, pres_th);

    if (pressure_sens_1 < 0.0) {
      pressure_sens_1 = 0.0;
    }

    if (pressure_sens_2 < 0.0) {
        pressure_sens_2 = 0.0;
    }

    if (pressure_sens_1 > max_adc) {
      pressure_sens_1 = max_adc;
    }

    if (pressure_sens_2 > max_adc) {
        pressure_sens_2 = max_adc;
    }

    // Maximum constant arm stiffness controls
    if (control_type == 1 || control_type == 2){

      k_stiff = k_max;

    }

    // Minimum constant arm stiffness controls
    if (control_type == 3 || control_type == 4){

      k_stiff = k_min;
      
    }

    // Variable arm stiffness controls, activates only if human handshake is present
    if ((control_type == 5 || control_type == 6) && (pressure_sens_1 > pres_th || pressure_sens_2 > pres_th)){
     
    k_stiff = k_min + K_P_stiff*(k_max-k_min)*(pressure_sens_1 + pressure_sens_2)/(2*max_adc);

    }

    // Handshake control
    if (pressure_sens_1 > pres_th || pressure_sens_2 > pres_th) {

      // if (flag_pressure == 0){

       flag_pressure = 1;
      //   feedback_activation = ros::Time::now();
      //   std::cout << "Handshake detected!" << std::endl;

      // }

      hand_cl = K_C*hand_max + K_P*hand_cl_max*(pressure_sens_1 + pressure_sens_2)/(2*max_adc);

      hand_cl = speedSaturation(hand_cl, hand_cl_old, cl_th);
      //hand_cl = lowPassAveraging(hand_cl);

      if (hand_cl > hand_max) {
        hand_cl = hand_max;
        }
      }

      else hand_cl = 0;

    // if (flag_pressure == 1 && (feedback_activation + pressure_latency <= ros::Time::now())) {

    flag_pressure = 0;
    //   hand_cl       = 0;

    //   std::cout << "Handshake lost..." << std::endl;

    // }

    // std::cout << "The hand cl in callback is " << hand_cl/19000 << "." << std::endl;

    pressure_sens_1_old = pressure_sens_1;
    pressure_sens_2_old = pressure_sens_2;
    hand_cl_old = hand_cl;
}

/*---------------------------------------------------------------------*
* FILTER FEEDBACK (EE POSITION) UPDATE                                 *
*                                                                      *
*----------------------------------------------------------------------*/

void pose_hatCallback(const geometry_msgs::Pose& msg)
{
  pos_hat_x = msg.position.x;
  pos_hat_y = msg.position.y;
  pos_hat_z = msg.position.z;
  pos_hat_quat_x = msg.orientation.x;
  pos_hat_quat_y = msg.orientation.y;
  pos_hat_quat_z = msg.orientation.z;
  pos_hat_quat_w = msg.orientation.w;
}

void poseD_hatCallback(const geometry_msgs::Twist& msg)
{
  posD_hat = msg.linear.z;
}

bool run_handshake_control(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  // Setting the run bool
  service_called = req.data;
  exp_begin = ros::Time::now();     // Resetting time for timeout
  res.success = true;
  res.message = "Done!";
  return true;
}

/*---------------------------------------------------------------------*
* MAIN                                                                 *
*                                                                      *
*----------------------------------------------------------------------*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "handshake_control_strategy_node");

  ros::NodeHandle node;

  control_type      = atoll(argv[1]);
  int subject_ID    = atoll(argv[2]);
  int subject_task  = atoll(argv[3]);
  int digit;
  double z_des_ctr;
  double zd_des_ctr;
  char term;

  while (!(control_type == 1 || control_type == 2 || control_type == 3 || control_type == 4 || control_type == 5 || control_type == 6)){

    std::cout << "Invalid control strategy paramether, please digit from 1 to 6" << std::endl;
    std::cin >> digit;
    if(scanf("%d%c", &digit, &term) != 2 || term != '\n'){

      return -1;

    }
    
    control_type = digit;

    }

    node.getParam("handshake_controller_type", control_type);
    std::cout << "Controller Type "<< control_type <<" Enabled" << std::endl;

    ros::Rate rate(100);
    exp_begin = ros::Time::now();

    std_msgs::Float64MultiArray stiffMatrixCmdMsg;
    Eigen::Matrix<double, 6, 6> kMatrix;
    std_msgs::Int16 control_ID_msg;
    std_msgs::Int16 ID_msg;
    std_msgs::Int16 task_msg;
    std_msgs::Int16 flag_msg;
    std_msgs::Int16 flag_pressure_msg;
    std_msgs::Int16 k_stiff_msg;
    //qb_interface::handRef hand_cl_msg;
    geometry_msgs::Twist vel_des_msg;
    geometry_msgs::PoseStamped posa_control_msg;
    ros::Time time_exp_msg;

    ros::Publisher pub_control_ee         = node.advertise<geometry_msgs::PoseStamped>("/panda_arm/equilibrium_pose",1); // Aggiungere topic
    ros::Publisher stiffMatrixCmdPub      = node.advertise<std_msgs::Float64MultiArray>("/panda_arm/desired_stiffness_matrix",1);
    ros::Publisher flagPub                = node.advertise<std_msgs::Int16>("handshake_feedback_flag_activation",1);
    ros::Publisher pub_pos_desD_ee        = node.advertise<geometry_msgs::Twist>("/handshake_EKF_desired_twist",1);
    //ros::Publisher pub_hand_cl            = node.advertise<qb_interface::handRef>("/qb_class/hand_ref",1);
    ros::Publisher pub_hand_cl            = node.advertise<trajectory_msgs::JointTrajectory>("/right_hand/joint_trajectory_controller/command",1);
    ros::Publisher pressure_feedback_pub  = node.advertise<std_msgs::Int16>("handshake_pressure_feedback_activation",1);
    ros::Publisher arm_stiffness_pub      = node.advertise<std_msgs::Int16>("handshake_current_arm_stiffness",1);
    ros::Publisher subject_ID_pub         = node.advertise<std_msgs::Int16>("handshake_subject_ID",1);
    ros::Publisher subject_task_pub       = node.advertise<std_msgs::Int16>("handshake_subject_task",1);
    ros::Publisher control_ID_pub         = node.advertise<std_msgs::Int16>("handshake_control_ID",1);
    ros::Publisher time_exp_pub           = node.advertise<std_msgs::Int16>("handshake_exp_time",1);
    ros::ServiceClient end_pub            = node.serviceClient<std_srvs::SetBool>("handshake_ending");

    ros::Subscriber sub_pos_hat   = node.subscribe("/handshake_EKF_controlled_pose",1,&pose_hatCallback);
    ros::topic::waitForMessage<geometry_msgs::Pose>("/handshake_EKF_controlled_pose", ros::Duration(5.0));
    ros::Subscriber sub_posD_hat  = node.subscribe("/handshake_controlled_twist",1,&poseD_hatCallback);
    ros::Subscriber sub_qb_adc    = node.subscribe("/qb_class_imu/adc",1,&qb_adcCallback); 

    ros::ServiceServer run_client = node.advertiseService("call_handshake_control", run_handshake_control);

    //ros::Subscriber sub_qb_adc    = node.subscribe("/qb_class_imu/adc",1,&Arm_Stiffness_Callback);   
         
    while (ros::ok()){
      //Spinning once to get messages from topics
      ros::spinOnce();

      if (service_called) { 
       
         if (kbhit()){
          (void)getch();

          if (flag == 1){
            flag = 0;
            std::cout << "Filter Feedback Not Enabled; press a key to enable it" << std::endl;
          }
          else {
            flag = 1;
            std::cout << "Filter Feedback Enabled, waiting for pressure signals; press a key to disable it" << std::endl;

          }
        }


      // EKF feedback on ee z position
        
        if (control_type == 1 || control_type == 3 || control_type == 5 && flag == 1 && flag_pressure == 1){

         z_des_ctr = pos_hat_z;
         zd_des_ctr = 0; 

       }

       else if (control_type == 2 || control_type == 4 || control_type == 6 || flag == 0)
       {

         z_des_ctr = 0.3;    
         zd_des_ctr = 0; 

       }


       vel_des_msg.linear.z = zd_des_ctr;

       if (z_des_ctr < 0.1)
        z_des_ctr = 0.1;

      if (z_des_ctr > 0.9)
        z_des_ctr = 0.9;

      //vel_des_msg.linear.z = EKF_feedback(control_type, );

      posa_control_msg.pose.position.x    = pos_hat_x;
      posa_control_msg.pose.position.y    = pos_hat_y;
      posa_control_msg.pose.position.z    = z_des_ctr;
      posa_control_msg.pose.orientation.x = pos_hat_quat_x;
      posa_control_msg.pose.orientation.y = pos_hat_quat_y;
      posa_control_msg.pose.orientation.z = pos_hat_quat_z;
      posa_control_msg.pose.orientation.w = pos_hat_quat_w;

      kMatrix = Eigen::MatrixXd::Zero(6,6);
      kMatrix.topLeftCorner(3, 3)      = k_stiff * Eigen::MatrixXd::Identity(3, 3);
      kMatrix.bottomRightCorner(3, 3)  = k_stiff * Eigen::MatrixXd::Identity(3, 3) / 20;

      stiffMatrixCmdMsg.data.clear();

      for (int j = 0; j < 36; j++)
      {
        stiffMatrixCmdMsg.data.push_back(kMatrix(j));
      }

      stiffMatrixCmdPub.publish(stiffMatrixCmdMsg);

      control_ID_msg.data = control_type;
      control_ID_pub.publish(control_ID_msg);

      ID_msg.data = subject_ID;
      subject_ID_pub.publish(ID_msg);

      task_msg.data = subject_task;
      subject_task_pub.publish(task_msg);

      flag_msg.data = flag;
      flagPub.publish(flag_msg);

      // hand_cl_msg.closure.clear();
      // hand_cl_msg.closure.push_back(hand_cl);
      // pub_hand_cl.publish(hand_cl_msg);
      
      // Creating traj msg for hand controller
      hand_cl_msg.points.clear();
      hand_cl_msg.joint_names.clear();
      hand_cl_msg.joint_names.push_back("right_hand_synergy_joint");

      trajectory_msgs::JointTrajectoryPoint start_point;
      start_point.time_from_start = ros::Duration(1.0/50.0);

      start_point.positions.clear();
      start_point.positions.push_back(hand_cl/19000);

      // std::cout << "The hand cl is " << hand_cl/19000 << "." << std::endl;

      hand_cl_msg.points.push_back(start_point);

      // Publishing to hand controller
      pub_hand_cl.publish(hand_cl_msg);

      flag_pressure_msg.data = flag_pressure;
      pressure_feedback_pub.publish(flag_pressure_msg);
      
      k_stiff_msg.data = k_stiff;
      arm_stiffness_pub.publish(k_stiff_msg);

      pub_pos_desD_ee.publish(vel_des_msg);
      pub_control_ee.publish(posa_control_msg);

      // ROS_WARN_STREAM("Pose to publish \n" << posa_control_msg);
      // int arc;
      // std::cin >> arc;

      exp_time = ros::Time::now();
      time_exp_msg = exp_time;
      time_exp_pub.publish(time_exp_msg);

      if (exp_begin + exp_finish <= ros::Time::now()){

        // hand_cl_msg.closure.clear();
        // hand_cl_msg.closure.push_back(0.0);
        // pub_hand_cl.publish(hand_cl_msg);

        // Creating traj msg for hand controller
        hand_cl_msg.points.clear();
        hand_cl_msg.joint_names.clear();
        hand_cl_msg.joint_names.push_back("right_hand_synergy_joint");

        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.time_from_start = ros::Duration(1.0/50.0);

        start_point.positions.clear();
        start_point.positions.push_back(0.0);

        hand_cl_msg.points.push_back(start_point);

        // Publishing to hand controller
        pub_hand_cl.publish(hand_cl_msg);

        ros::Duration(1).sleep();  // per dare tempo ai publisher di avviarsi
        //ros::shutdown();

        std_srvs::SetBool end_srv; end_srv.request.data = true;
        end_pub.call(end_srv);


        service_called = false;
        exp_begin = ros::Time::now();

        // Resetting stiffness matrix to minimum value
        kMatrix = Eigen::MatrixXd::Zero(6,6);
        kMatrix.topLeftCorner(3, 3)      = k_min * Eigen::MatrixXd::Identity(3, 3);
        kMatrix.bottomRightCorner(3, 3)  = k_min * Eigen::MatrixXd::Identity(3, 3) / 20;

        stiffMatrixCmdMsg.data.clear();

        for (int j = 0; j < 36; j++)
        {
          stiffMatrixCmdMsg.data.push_back(kMatrix(j));
        }

        stiffMatrixCmdPub.publish(stiffMatrixCmdMsg);

      }

    }

    ros::spinOnce();
    rate.sleep();

  }
    

}

