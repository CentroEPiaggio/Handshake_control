
#include <trajectory_msgs/JointTrajectory.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include "class_loader/class_loader.h"
#include <eigen3/Eigen/Eigen>

#include <fstream> 
#include "ros/ros.h"
#include <ros/node_handle.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
//#include <lwr_controllers/PoseRPY.h>
#include <sensor_msgs/JointState.h>
#include <kdl/tree.hpp>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <urdf/model.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <handshake_ekf/StateVec.h>
//#include <filter/StateVec.h>

using namespace KDL;

#include "handshake_EKF_functions.h"
//#include "filter/ClassArm.h"


using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using namespace std;

// posa e orientazione ee 
double x_measure = 0.66;
double y_measure = 0.37;
double z_measure = 0.27;
Quaterniond r_quat = Quaterniond::Identity(); 



// double x_quat = -0.7;
// double y_quat = 0;
// double z_quat = 0;
// double w_quat = 0.7;


double z_hat =0;
double zd_hat= 0;


double z_des = z_measure;
double zd_des = 0;
int first_cy = 1;

void stiff_desCallback(const std_msgs::Float64MultiArray& msg){
  Kr = msg.data[0];
  Dr = 2*sqrt(Kr);
}


void poseCallback(const geometry_msgs::PoseStamped& msg)
{ 
  if (first_cy){
    x_measure = msg.pose.position.x;
    y_measure = msg.pose.position.y;
    r_quat.x() = msg.pose.orientation.x;
    r_quat.y() = msg.pose.orientation.y;
    r_quat.z() = msg.pose.orientation.z;
    r_quat.w() = msg.pose.orientation.w;
  }
  first_cy = 0;
  z_measure = msg.pose.position.z;

  // x_quat = msg.pose.orientation.x;
  // y_quat = msg.pose.orientation.y;
  // z_quat = msg.pose.orientation.z;
  // w_quat = msg.pose.orientation.w;
}

void pose_desCallback(const geometry_msgs::PoseStamped& msg)
{
  z_des = msg.pose.position.z;
}

void poseD_desCallback(const geometry_msgs::Twist& msg)
{
  zd_des = msg.linear.z;
}



int main(int argc, char **argv){

    ros::init(argc, argv, "handshake_EKF_node");
    ros::NodeHandle node;

    ros::Rate rate(100);


    r_quat.x() = -0.73;
    r_quat.y() = 0.12;
    r_quat.z() = 0.12;
    r_quat.w() = 0.65;
    r_quat = r_quat.normalized();

    //ros::Publisher pub = node.advertise<lwr_controllers::PoseRPY>("/right_arm/one_task_inverse_kinematics_controller/command",1000);
    ros::Publisher Stato = node.advertise<handshake_ekf::StateVec>("/handshake_EKF_state",10);
    //ros:: Publisher pub_teleOp = node.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command",100);
    ros:: Publisher pub_contr = node.advertise<geometry_msgs::Pose>("/handshake_EKF_controlled_pose",100);
    ros:: Publisher pub_contrD = node.advertise<geometry_msgs::Twist>("/handshake_EKF_controlled_twist",100);
   
    // Da modificare il topic
    ros::Subscriber sub_pos_ee = node.subscribe("/panda_arm/franka_ee_pose",1,&poseCallback);
    ros::Subscriber sub_pos_des_ee = node.subscribe("/panda_arm/equilibrium_pose",1,&pose_desCallback);
    ros::Subscriber sub_posD_des_ee = node.subscribe("/handshake_EKF_desired_twist",1,&poseD_desCallback);
    ros::Subscriber sub_stiff_des_ee = node.subscribe("/panda_arm/desired_stiffness_matrix",1,&stiff_desCallback);


	 // lwr_controllers::PoseRPY msg;
    geometry_msgs::Pose pos_hat_msg;
    geometry_msgs::Twist posD_hat_msg;

    // Inizialiazzazione eventuali publisher

    ros::Duration(1).sleep();  // per dare tempo ai publisher di avviarsi
    for(int j = 0; j<2; j++){
    	ros::spinOnce();
   		rate.sleep();
    }

    // Definisco il vettore di stato X: dimensione 18
    VectorXd X(6);

    // Definisco matrice P 18x18
       MatrixXd P(6,6);
    // Matrice Q
       MatrixXd Q(6,16);
    // Matrice R
       VectorXd R(1);
    // Matrice K
       MatrixXd K(6,1);

    // Inizializzazione
    VectorStateInit(X);
    PInit(P);
    QInit(Q);
    R(0) = 0.0005;
 
    // Alcune definizioni di variabili

    double T_measure = 0.01; // ogni quanto arrivano le misure

    double z;
    double zd;
   
    KDL::JntArray q_j, qd_j;
    q_j.resize(7);
    qd_j.resize(7);

    double x1;
    double x2;
    double x3;
    double x4;

   MatrixXd H(1,6);
   H = MatrixXd::Zero(1,6);
   H(0,4) = 1;

   MatrixXd I(6,6);
   I = MatrixXd::Identity(6,6);

   int flag = 0;
   KDL::Frame X_ee;
   // double z_measure;


    //ROS_INFO_STREAM("FINE \n" );

     for(int j = 0; j<10; j++){
      ros::spinOnce();
      rate.sleep();
    }

    double m = 1;

// Calcolo il Jacobiano
   MatrixXd F(6,6);
   


   double z_uscita;
   handshake_ekf::StateVec stato_msg;
   double time_filter = 0;
   double roll_base;
   double pitch_base;
   double yaw_base;
   double roll_world;
   double pitch_world;
   double yaw_world;
   double quat_x,quat_y,quat_z,quat_w;

  Eigen::Matrix<double,6,1> config;
  Eigen::Matrix<double,3,1> pos;
  Eigen::Matrix<double,3,1> orien;

  double T_ant = 0.1; // T di anticipo per uscita filtro



  // INSERIRE INGRESSI-> z e z_des


   // 2 cicli for, per ogni misura ricevuta la dinamica è calcolata N volte
    //cout << "pippo" << endl;
   while (ros::ok()){
     	        
           // Estraggo componenti vettore di stato

           x1 = X[0];
           x2 = X[1];
           x3 = X[2];
           x4 = X[3];
            z = X[4];
           zd = X[5];


           	// Dinamica
           	x1 = x1 + T*x2*x4;
           	x2 = x2 - T*x1*x4;
           	x3 = x3;
           	x4 = x4;
           	z = z +T*zd;
            zd = zd - (T/m)*(Bsm*(zd - x4*x2) + Kel*(z - x1 - x3) + Dr*(zd - zd_des) + Kr*(z - z_des));
            
   
           	// aggiorno il vettore di stato
           	X[0] = x1;
           	X[1] = x2;
           	X[2] = x3;
           	X[3] = x4;
           	X[4] = z;
            X[5] = zd;

           	//ROS_INFO_STREAM("X\n" << X);

           	// Proiezione in avanti di P
            F_matrix(F,X,m);

            //cout << "K: " << Kr << "  " << "Dr: " << Dr << endl;

           	P = F*P*F.transpose() + Q;
           
            // Calcolo K 
            K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
               
            // Update
            X = X + K*(z_measure -z);
            P =  (I-K*H)*P;


    //ROS_INFO_STREAM("z\n" << z_uscita);

    stato_msg.header.stamp = ros::Time::now();
    stato_msg.x1 = X(0); 
    stato_msg.x2 = X(1); 
    stato_msg.x3 = X(2); 
    stato_msg.x4 = X(3); 
    stato_msg.z = X(4); 
    stato_msg.zd = X(5); 
    Stato.publish(stato_msg);

   //time_filter = time_filter + T_measure;

    z_hat = X(0) + X(2) + T_ant*X(1)*X(3);
    zd_hat = X(1)*X(3) - T_ant*X(0)*X(3)*X(3);

    pos_hat_msg.position.x = x_measure;
    pos_hat_msg.position.y = y_measure;
    pos_hat_msg.position.z = z_hat;
    pos_hat_msg.orientation.x = r_quat.x();
    pos_hat_msg.orientation.y = r_quat.y();
    pos_hat_msg.orientation.z = r_quat.z();
    pos_hat_msg.orientation.w = r_quat.w();

    // pos_hat_msg.orientation.x = x_quat;
    // pos_hat_msg.orientation.y = y_quat;
    // pos_hat_msg.orientation.z = z_quat;
    // pos_hat_msg.orientation.w = w_quat;

    posD_hat_msg.linear.z = zd_hat;


    pub_contr.publish(pos_hat_msg);
    pub_contrD.publish(posD_hat_msg);
    

     

/*
   

   if(time_filter > 4)
   {

      if(z_uscita > 1.4)
      {
        z_uscita = 1.4;
      }

      if(z_uscita < 0.6)
      {
        z_uscita = 0.6;
      }


    //ROS_INFO_STREAM("z_measure\n" << z_measure);
            z_des = X(0) + X(2) + T_ant*X(1)*X(3);
            zd_des = X(1)*X(3) - T_ant*X(0)*X(3)*X(3);
            z_uscita = z_des; 

    da.x_right_.M.GetEulerZYX(yaw_base,pitch_base,roll_base);
    da.x_right_w.M.GetEulerZYX(yaw_world,pitch_world,roll_world);
    da.x_right_w.M.GetQuaternion(quat_x,quat_y,quat_z,quat_w);

    pos << da.x_right_w.p(0), da.x_right_w.p(1), z_uscita;
    orien << roll_world, pitch_world, yaw_world;

   
    msg1.position.x = da.x_right_w.p(0);
    msg1.position.y = da.x_right_w.p(1);
    msg1.position.z = z_uscita;
    msg1.orientation.x = quat_x;
    msg1.orientation.y = quat_y;
    msg1.orientation.z = quat_z;
    msg1.orientation.w = quat_w;

    //pub.publish(msg);
    pub_teleOp.publish(msg1);

   }
*/

   ros::spinOnce();
   rate.sleep();



   }

   
}
