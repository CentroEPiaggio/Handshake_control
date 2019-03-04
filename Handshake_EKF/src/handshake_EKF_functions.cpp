#include "handshake_EKF_functions.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <math.h>


#include <trajectory_msgs/JointTrajectory.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>



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

//#include "filter/ClassArm.h"

using namespace KDL;


using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using namespace std;


// Definisco alcune variabili globali utili per calcolo matrici dinamiche
// queste non saranno necessarie col Kuka
double PI_g = 3.14159;


// Parametri molla-smorzatore
double Kel = 500;
double Bsm = 2*sqrt(Kel);



// Costanti molla-smorzatore robot
double Kr = 200;
double Dr = 2*sqrt(Kr);


// Tempo di campionamento
double T = 0.01;

// FUNZIONI

Eigen::Matrix<double,3,3> quat_rot(double x, double y, double z, double w) {
  Eigen::Matrix<double,3,3> temp;
  temp << 1-2*pow(y,2)-2*pow(z,2),             2*x*y-2*w*z,             2*x*z+2*w*y,
                  2*x*y+2*w*z, 1-2*pow(x,2)-2*pow(z,2),             2*z*y-2*w*x,
                  2*x*z-2*w*y,             2*z*y+2*w*x, 1-2*pow(x,2)-2*pow(y,2);
  return temp;
}
/*
// Prende in ingesso posizione e orientazione in world e li restituisce nella base del braccio
  Eigen::Matrix<double,6,1> World2Base(Eigen::Matrix<double,3,1> pos_w, Eigen::Matrix<double,3,1> or_w ){

  KDL::Rotation world2base;
  KDL::Rotation world2ee;
  Eigen::Matrix<double,3,1> vect_pos;
  Eigen::Matrix<double,3,1> vect_or; // angoli di roll pitch e yaw


  Eigen::Matrix<double,3,1> origin_base;
  origin_base << 0.77, 0.801, 1.607; 

  double t1,t2,t3,t4;
  Eigen::Matrix<double,3,3> rot_mat;
  double s1,s2,s3,s4;
  Eigen::Matrix<double,3,3> my_rot;


  world2base = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
  world2ee =  KDL::Rotation::RPY(or_w(0),or_w(1),or_w(2));

  world2base.GetQuaternion(t1,t2,t3,t4);
  rot_mat = quat_rot(t1,t2,t3,t4);
  world2ee.GetQuaternion(s1,s2,s3,s4);
  my_rot = quat_rot(s1,s2,s3,s4);


  vect_pos = pos_w - origin_base;
  vect_pos = rot_mat.transpose()*vect_pos; // posizione ee in base braccio

  my_rot = rot_mat.transpose()*my_rot;


  world2ee(0,0) =  my_rot(0,0);
    world2ee(0,1) =  my_rot(0,1);
    world2ee(0,2) =  my_rot(0,2);
    world2ee(1,0) =  my_rot(1,0);
    world2ee(1,1) =  my_rot(1,1);
    world2ee(1,2) =  my_rot(1,2);
    world2ee(2,0) =  my_rot(2,0);
    world2ee(2,1) =  my_rot(2,1);
    world2ee(2,2) =  my_rot(2,2);

    world2ee.GetEulerZYX(vect_or(2),vect_or(1),vect_or(0));


    Eigen::Matrix<double,6,1> var;
    var << vect_pos,
        vect_or;

    return var;

}
*/
/*
int check(VectorXd& goal_pos, VectorXd& goal_or, DualArms& da)
{
  VectorXd err_pos(3);
  VectorXd err_or(3);
  VectorXd pos_w(3);
  pos_w(0) = da.x_right_w.p(0);
  pos_w(1) = da.x_right_w.p(1);
  pos_w(2) = da.x_right_w.p(2);
  VectorXd or_w(3);
  da.x_right_w.M.GetEulerZYX(or_w(0),or_w(1),or_w(2));
  err_pos = goal_pos - pos_w;
  err_or = goal_or - pos_w;

  int temp = 0;

  if(err_pos(0)<0.1 && err_pos(1)<0.1,err_pos(2)<0.1)
  {
    temp = 1;
  }

  return temp;
}

*/
/*
//Definisco funzione che calcola tau_m
void comput_tau_m(VectorXd& x, VectorXd& tau_m, MatrixXd& Jac, KDL::Frame& X_ee ) {

//Vettore di stato
 double x1 = x(0);
 double x2 = x(1);
 double x3 = x(2);
 double x4 = x(3);
 double q1 = x(4);
 double q2 = x(5);
 double q3 = x(6);
 double q4 = x(7);
 double q5 = x(8);
 double q6 = x(9);
 double q7 = x(10);
 double qd1 = x(11);
 double qd2 = x(12);
 double qd3 = x(13);
 double qd4 = x(14);
 double qd5 = x(15);
 double qd6 = x(16);
 double qd7 = x(17);
 
 VectorXd qd(7);
 VectorXd q(7);
 

 q << q1, q2, q3, q4, q5, q6, q7;
 qd << qd1, qd2, qd3, qd4, qd5, qd6, qd7;


 KDL::JntArray q_joint;
 q_joint.data = q;

// NOTA: SCRITTE IN WORLD; SARANNO DA CAMBIARE
// Carrello
double x_c = 1.60;
double xd_c = 0;
double y_c  = 0.8;
double yd_c = 0;
double z_c = x1 + x3;
double zd_c = x2*x4;
double phi_c = 0;
double theta_c = 0;
double psi_c = 0;
double phid_c = 0;
double thetad_c = 0;
double psid_c = 0;


// Jacobiano
//MatrixXd Jac;
//da.Jacobiano(q_joint);

// Robot
//KDL::Frame X_ee;
//X_ee = da.ForwardKinematics_W(q_joint);

double x_r = X_ee.p(0);
double y_r = X_ee.p(1);
double z_r = X_ee.p(2);


double phi_r;
double theta_r;
double psi_r;

X_ee.M.GetEulerZYX(psi_r,theta_r,phi_r);

VectorXd twist(6);
twist = Jac*qd;


double xd_r = twist(0);
double yd_r = twist(1);
double zd_r = twist(2);
double phid_r = twist(3);
double thetad_r = twist(4);
double psid_r = twist(5);

// Wrench W 
double fx = (x_c-x_r)*Kel + (xd_c-xd_r)*Bsm;
double fy = (y_c-y_r)*Kel + (yd_c-yd_r)*Bsm;
double fz = (z_c-z_r)*Kel + (zd_c-zd_r)*Bsm;
double mx = (phi_c-phi_r)*Ktq + (phid_c-phid_r)*Btq;
double my = (theta_c-theta_r)*Ktq + (thetad_c-thetad_r)*Btq;
double mz = (psi_c-psi_r)*Ktq + (psid_c-psid_r)*Btq;

VectorXd W(6);
W << fx, fy, fz, mx, my, mz;

tau_m =Jac.transpose()*W;
//ROS_INFO_STREAM("tau_m \n" << tau_m);

return;
}
*/


// Inizializzazione filtro
void VectorStateInit(VectorXd& VectState) {

 // La dimensione del vettore di stato è 18
/*
 VectState(0) = 0.5;     // x1
 VectState(1) = 2;       // x2
 VectState(2) = 0.3;     // x3
 VectState(3) = 8;       // x4
 VectState(4) = -3*PI_g/8; // q1
 VectState(5) = PI_g/8;    // q2
 VectState(6) = PI_g/4;    // q3
 VectState(7) = 0.1;       // q4
 VectState(8) = 0.1;       // q5
 VectState(9) = 0.1;       // q6
 VectState(10) = 0.1;      // q7
 VectState(11) = 0;      // qd1
 VectState(12) = 0;      // qd2
 VectState(13) = 0;      // qd3
 VectState(14) = 0;      // qd4
 VectState(15) = 0;      // qd5
 VectState(16) = 0;      // qd6
 VectState(17) = 0;      // qd7
*/

 VectState(0) = 0.1;     // x1
 VectState(1) = 0;       // x2
 VectState(2) = 0.3;     // x3
 VectState(3) = 8;       // x4
 VectState(4) = 0.3; // q1  era 0.8
 VectState(5) = 0;    // q2

}

void PInit( MatrixXd& P_mat) {
 double par = 100;
 P_mat = par*MatrixXd::Identity(6,6);
}

void QInit(MatrixXd& Q_mat) {
 Q_mat = MatrixXd::Zero(6,6);

 double q_x1 = 1;//= 1;
 double q_x2 = 1;//= 1;
 double q_x3 = 0.05;
 double q_x4 = 6;
 double q_z = 0.15;
 double q_zd = 0.2;

 Q_mat(0,0) = q_x1;
 Q_mat(1,1) = q_x2;
 Q_mat(2,2) = q_x3;
 Q_mat(3,3) = q_x4;
 Q_mat(4,4) = q_z;
 Q_mat(5,5) = q_zd;
 
}



void F_matrix(MatrixXd& F, VectorXd& X, double m) {


       //Estraggo componenti vettore di stato
        double x1 = X(0);
        double x2 = X(1);
        double x3 = X(2);
        double x4 = X(3);
        double z = X(4);
        double zd = X(5);

        F = MatrixXd::Zero(6,6);
        F(0,0) = 1;
        F(1,1) = 1;
        F(2,2) = 1;
        F(3,3) = 1;
        F(4,4) = 1;
        F(4,5) = T;
        F(5,0) = T*Kel/m;
        F(5,2) = T*Kel/m;
        F(5,4) = -T*(Kel+Kr)/m;
        F(5,5) = 1-T*(Bsm+Dr)/m;


        F(0,1) = T*x4;
        F(0,3) = T*x2;
        F(1,0) = -T*x4;
        F(1,3) = -T*x1;
        F(5,1) = (T*Bsm/m)*x4;
        F(5,3) = (T*Bsm/m)*x2;



        //ROS_INFO_STREAM("F \n" << F);

       return;
       
   }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void comput_tau_mm(VectorXd& x, VectorXd& tau_m, DualArms& da) {

//Vettore di stato
 double x1 = x(0);
 double x2 = x(1);
 double x3 = x(2);
 double x4 = x(3);
 double q1 = x(4);
 double q2 = x(5);
 double q3 = x(6);
 double q4 = x(7);
 double q5 = x(8);
 double q6 = x(9);
 double q7 = x(10);
 double qd1 = x(11);
 double qd2 = x(12);
 double qd3 = x(13);
 double qd4 = x(14);
 double qd5 = x(15);
 double qd6 = x(16);
 double qd7 = x(17);
 
 VectorXd qd(7);
 VectorXd q(7);
 

 q << q1, q2, q3, q4, q5, q6, q7;
 qd << qd1, qd2, qd3, qd4, qd5, qd6, qd7;


 KDL::JntArray q_joint;
 q_joint.data = q;

// NOTA: SCRITTE IN WORLD; SARANNO DA CAMBIARE
// Carrello
double x_c = 1.60;
double xd_c = 0;
double y_c  = 0.8;
double yd_c = 0;
double z_c = x1 + x3;
double zd_c = x2*x4;
double phi_c = 0;
double theta_c = 0;
double psi_c = 0;
double phid_c = 0;
double thetad_c = 0;
double psid_c = 0;


// Jacobiano
MatrixXd Jac;
da.Jacobiano(q_joint);

// Robot
KDL::Frame X_ee;
X_ee = da.ForwardKinematics_W(q_joint);

double x_r = X_ee.p(0);
double y_r = X_ee.p(1);
double z_r = X_ee.p(2);


double phi_r;
double theta_r;
double psi_r;

X_ee.M.GetEulerZYX(psi_r,theta_r,phi_r);

VectorXd twist(6);
twist = da.J_mat.data*qd;


double xd_r = twist(0);
double yd_r = twist(1);
double zd_r = twist(2);
double phid_r = twist(3);
double thetad_r = twist(4);
double psid_r = twist(5);

// Wrench W = [fx;fy;mz]
double fx = (x_c-x_r)*Kel + (xd_c-xd_r)*Bsm;
double fy = (y_c-y_r)*Kel + (yd_c-yd_r)*Bsm;
double fz = (z_c-z_r)*Kel + (zd_c-zd_r)*Bsm;
double mx = (phi_c-phi_r)*Ktq + (phid_c-phid_r)*Btq;
double my = (theta_c-theta_r)*Ktq + (thetad_c-thetad_r)*Btq;
double mz = (psi_c-psi_r)*Ktq + (psid_c-psid_r)*Btq;

VectorXd W(6);
W << fx, fy, fz, mx, my, mz;

tau_m = da.J_mat.data.transpose()*W;
//ROS_INFO_STREAM("tau_m \n" << tau_m);

return;
}


void F_matrixx(MatrixXd& F, VectorXd& X, VectorXd& dX, DualArms& da, MatrixXd& B_matrix, MatrixXd& B_matrix_inv, VectorXd& Cqd_vector,VectorXd& G_vector) {

       //Vettore di stato incrementato
       VectorXd X_inc(18);


       VectorXd q(7);
       VectorXd qd(7);
       VectorXd tau(7);

       VectorXd tau_m(7);


       //Estraggo componenti vettore di stato
        double x1 = X(0);
        double x2 = X(1);
        double x3 = X(2);
        double x4 = X(3);
        double q1 = X(4);
        double q2 = X(5);
        double q3 = X(6);
        double q4 = X(7);
        double q5 = X(8);
        double q6 = X(9);
        double q7 = X(10);
        double qd1 = X(11);
        double qd2 = X(12);
        double qd3 = X(13);
        double qd4 = X(14);
        double qd5 = X(15);
        double qd6 = X(16);
        double qd7 = X(17);
  

        double dx1 = dX(0);
        double dx2 = dX(1);
        double dx3 = dX(2);
        double dx4 = dX(3);
        double dq1 = dX(4);
        double dq2 = dX(5);
        double dq3 = dX(6);
        double dq4 = dX(7);
        double dq5 = dX(8);
        double dq6 = dX(9);
        double dq7 = dX(10);
        double dqd1 = dX(11);
        double dqd2 = dX(12);
        double dqd3 = dX(13);
        double dqd4 = dX(14);
        double dqd5 = dX(15);
        double dqd6 = dX(16);
        double dqd7 = dX(17);


       q << q1, q2, q3, q4, q5, q6, q7;

       qd << qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       tau = da.tau_right_meas_.data;

        KDL::JntArray q_joint, qd_joint;
        q_joint.data = q;
        qd_joint.data = qd;

        VectorXd q_inc(7);
        VectorXd qd_inc(7);


       KDL::JntArray q_joint_inc, qd_joint_inc;
       q_joint_inc.resize(7);
       qd_joint_inc.resize(7);
      

       

      VectorXd tau_m_inc(7);  //Vettore di appoggio per calcolare derivate numeriche

       MatrixXd B_inv_inc(7,7);

       MatrixXd B_inc(7,7);

       MatrixXd C_inc(7,7);

       VectorXd G_inc(7);

       VectorXd a(7);


       //Calcolo B,C e G: per le prime quattro variabili lo faccio una volta, per le altre mi costruisco delle variabili di appoggio

       da.B_calc(q_joint);
       da.G_calc(q_joint);
       da.C_calc(q_joint,qd_joint);


       X << x1, x2, x3, x4, q1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

//first column of the Jacobian matrix(derivative respect to x1)

       X_inc << x1+dx1, x2, x3, x4, q1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;


       //Calcolo tau_m per il vettore con incremento e lo salvo nel vettore di appoggio
        comput_tau_mm(X_inc,tau_m_inc,da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);
      



       F(0,0) = ((x1 + dx1 + T*x2*x4) - (x1 + T*x2*x4))/dx1;

       F(1,0) = ((x2 - T*(x1 + dx1)*x4) - (x2 -T*x1*x4))/dx1;
       
       F(2,0) = 0;

       F(3,0) = 0;

       F(4,0) = 0;

       F(5,0) = 0;

       F(6,0) = 0;

       F(7,0) = 0;

       F(8,0) = 0;

       F(9,0) = 0;

       F(10,0) = 0;

       VectorXd v(7);

       v = ((qd + T*da.B_mat.data.inverse()*(tau + tau_m_inc - da.C_vect.data - da.G_vectt.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dx1;
       
      

       //v = ((B_inv*(tau + tau_m_inc -C*qd - G)) - (B_inv*(tau + tau_m -C*qd - G)))/dx1;


       F(11,0)  = v(0);

       F(12,0)  = v(1);

       F(13,0)  = v(2);

       F(14,0)  = v(3);

       F(15,0)  = v(4);

       F(16,0)  = v(5);

       F(17,0)  = v(6);

     // ROS_INFO_STREAM("F1_old) \n" << F.block(0,0,18,1));

//second column of the Jacobian matrix(derivative respect to x2)

       X_inc << x1, x2 + dx2, x3, x4, q1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

      // v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento e lo salvo nel vettore di appoggio
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       F(0,1) = ((x1 + T*(x2 + dx2)*x4) - (x1 + T*x2*x4))/dx2;

       F(1,1) = ((x2 + dx2 - T*x1*x4) - (x2 - T*x1*x4))/dx2;

       F(2,1) = 0;

       F(3,1) = 0;

       F(4,1) = 0;

       F(5,1) = 0;

       F(6,1) = 0;

       F(7,1) = 0;

       F(8,1) = 0;

       F(9,1) = 0;

       F(10,1) = 0;

       v = ((qd + T*da.B_mat.data.inverse()*(tau + tau_m_inc - da.C_vect.data - da.G_vectt.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dx2;

       //v = ((B_inv*(tau + tau_m_inc -C*qd - G)) - (B_inv*(tau + tau_m -C*qd - G)))/dx2;

       F(11,1)  = v(0);

       F(12,1)  = v(1);

       F(13,1)  = v(2);

       F(14,1)  = v(3);

       F(15,1)  = v(4);

       F(16,1)  = v(5);

       F(17,1)  = v(6);

       //ROS_INFO_STREAM("F1_old) \n" << F.block(0,1,18,1));

//third column of the Jacobian matrix(derivative respect to x3)
       X_inc << x1, x2 , x3 + dx3, x4, q1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;


       //Calcolo tau_m per il vettore con incremento e lo salvo nel vettore di appoggio
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       //v << 0, 0, 0, 0, 0, 0, 0;

       F(0,2) = 0;
       F(1,2) = 0;
       F(2,2) = 1;
       F(3,2) = 0;

       F(4,2) = 0;

       F(5,2) = 0;

       F(6,2) = 0;

       F(7,2) = 0;

       F(8,2) = 0;

       F(9,2) = 0;

       F(10,2) = 0;

      v = ((qd + T*da.B_mat.data.inverse()*(tau + tau_m_inc - da.C_vect.data - da.G_vectt.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dx3;

       F(11,2)  = v(0);

       F(12,2)  = v(1);

       F(13,2)  = v(2);

       F(14,2)  = v(3);

       F(15,2)  = v(4);

       F(16,2)  = v(5);

       F(17,2)  = v(6);

      //ROS_INFO_STREAM("F3_old) \n" << F.block(0,2,18,1));




//fourth column of the Jacobian matrix(derivative respect to x4)

       X_inc << x1, x2 , x3, x4 + dx4, q1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       //v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento e lo salvo nel vettore di appoggio
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);


       F(0,3) = 0;
       F(1,3) = 0;
       F(2,3) = 0;
       F(3,3) = 1;

       F(4,3) = 0;

       F(5,3) = 0;

       F(6,3) = 0;

       F(7,3) = 0;

       F(8,3) = 0;

       F(9,3) = 0;

       F(10,3) = 0;

      v = ((qd + T*da.B_mat.data.inverse()*(tau + tau_m_inc - da.C_vect.data - da.G_vectt.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dx4;

       F(11,3)  = v(0);

       F(12,3)  = v(1);

       F(13,3)  = v(2);

       F(14,3)  = v(3);

       F(15,3)  = v(4);

       F(16,3)  = v(5);

       F(17,3)  = v(6);

      //ROS_INFO_STREAM("F4_old) \n" << F.block(0,3,18,1));



//fifth column of the Jacobian matrix(derivative respect to q1)

       X_inc << x1, x2 , x3, x4 , q1 + dq1, q2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       //v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento e lo salvo nel vettore di appoggio
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);


       F(0,4) = 0;
       F(1,4) = 0;
       F(2,4) = 0;
       F(3,4) = 0;

       F(4,4) = ((q1 + dq1) + T*qd1 - (q1 + T*qd1))/dq1;
       F(5,4) = 0;
       F(6,4) = 0;
       F(7,4) = 0;
       F(8,4) = 0;
       F(9,4) = 0;
       F(10,4) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq1;

       F(11,4)  = v(0);

       F(12,4)  = v(1);

       F(13,4)  = v(2);

       F(14,4)  = v(3);

       F(15,4)  = v(4);

       F(16,4)  = v(5);

       F(17,4)  = v(6);

      // ROS_INFO_STREAM("F5_old) \n" << F.block(0,4,18,1));

       


//sixth column of the Jacobian matrix(derivative respect to q2)

       X_inc << x1, x2 , x3, x4 , q1, q2 + dq2, q3, q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


      // v << 0, 0, 0, 0, 0, 0, 0;


       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);


       F(0,5) = 0;
       F(1,5) = 0;
       F(2,5) = 0;
       F(3,5) = 0;

       F(4,5) = 0;
       F(5,5) = ((q2 + dq2) + T*qd2 - (q2 + T*qd2))/dq2;
       F(6,5) = 0;
       F(7,5) = 0;
       F(8,5) = 0;
       F(9,5) = 0;
       F(10,5) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq2;

       F(11,5)  = v(0);

       F(12,5)  = v(1);

       F(13,5)  = v(2);

       F(14,5)  = v(3);

       F(15,5)  = v(4);

       F(16,5)  = v(5);

       F(17,5)  = v(6);

       // ROS_INFO_STREAM("F6_old) \n" << F.block(0,5,18,1));

//seventh column of the Jacobian matrix(derivative respect to q3)

       X_inc << x1, x2 , x3, x4 , q1, q2, q3 + dq3,q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


      // v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       F(0,6) = 0;
       F(1,6) = 0;
       F(2,6) = 0;
       F(3,6) = 0;

       F(4,6) = 0;
       F(5,6) = 0;
       F(6,6) = ((q3 + dq3) + T*qd3 - (q3 + T*qd3))/dq3;
       F(7,6) = 0;
       F(8,6) = 0;
       F(9,6) = 0;
       F(10,6) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq3;

       F(11,6)  = v(0);

       F(12,6)  = v(1);

       F(13,6)  = v(2);

       F(14,6)  = v(3);

       F(15,6)  = v(4);

       F(16,6)  = v(5);

       F(17,6)  = v(6);

       //ROS_INFO_STREAM("F7_old) \n" << F.block(0,6,18,1));

//eight column of the Jacobian matrix(derivative respect to q4)

       X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4 + dq4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       //v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       F(0,7) = 0;
       F(1,7) = 0;
       F(2,7) = 0;
       F(3,7) = 0;

       F(4,7) = 0;
       F(5,7) = 0;
       F(6,7) = 0;
       F(7,7) = ((q4 + dq4) + T*qd4 - (q4 + T*qd4))/dq4;
       F(8,7) = 0;
       F(9,7) = 0;
       F(10,7) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq4;

       F(11,7)  = v(0);

       F(12,7)  = v(1);

       F(13,7)  = v(2);

       F(14,7)  = v(3);

       F(15,7)  = v(4);

       F(16,7)  = v(5);

       F(17,7)  = v(6);

        ROS_INFO_STREAM("F8_old) \n" << F.block(0,7,18,1));


//nineth column of the Jacobian matrix(derivative respect to q5)

       X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5 + dq5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       //v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       F(0,8) = 0;
       F(1,8) = 0;
       F(2,8) = 0;
       F(3,8) = 0;

       F(4,8) = 0;
       F(5,8) = 0;
       F(6,8) = 0;
       F(7,8) = 0;
       F(8,8) = ((q5 + dq5) + T*qd5 - (q5 + T*qd5))/dq5;
       F(9,8) = 0;
       F(10,8) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq5;

       F(11,8)  = v(0);

       F(12,8)  = v(1);

       F(13,8)  = v(2);

       F(14,8)  = v(3);

       F(15,8)  = v(4);

       F(16,8)  = v(5);

       F(17,8)  = v(6);

       // ROS_INFO_STREAM("F9_old) \n" << F.block(0,8,18,1));

//tenth column of the Jacobian matrix(derivative respect to q6)

       X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6 + dq6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       //v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);

       F(0,9) = 0;
       F(1,9) = 0;
       F(2,9) = 0;
       F(3,9) = 0;

       F(4,9) = 0;
       F(5,9) = 0;
       F(6,9) = 0;
       F(7,9) = 0;
       F(8,9) = 0;
       F(9,9) = ((q6 + dq6) + T*qd6 - (q6 + T*qd6))/dq6;
       F(10,9) = 0;

       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq6;

       F(11,9)  = v(0);

       F(12,9)  = v(1);

       F(13,9)  = v(2);

       F(14,9)  = v(3);

       F(15,9)  = v(4);

       F(16,9)  = v(5);

       F(17,9)  = v(6);

       ROS_INFO_STREAM("F10_old) \n" << F.block(0,9,18,1));


//eleventh column of the Jacobian matrix(derivative respect to q7)

       X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7 + dq7, qd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


      // v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);
        

       F(0,10) = 0;
       F(1,10) = 0;
       F(2,10) = 0;
       F(3,10) = 0;

       F(4,10) = 0;
       F(5,10) = 0;
       F(6,10) = 0;
       F(7,10) = 0;
       F(8,10) = 0;
       F(9,10) = 0;
       F(10,10) = ((q7 + dq7) + T*qd7 - (q7 + T*qd7))/dq7;

      
       v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dq7;


       F(11,10)  = v(0);

       F(12,10)  = v(1);

       F(13,10)  = v(2);

       F(14,10)  = v(3);

       F(15,10)  = v(4);

       F(16,10)  = v(5);

       F(17,10)  = v(6);

       //ROS_INFO_STREAM("F11_old) \n" << F.block(0,10,18,1));
        

//twelfth column of the Jacobian matrix(derivative respect to qd1)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1 + dqd1, qd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,11) = 0;
       F(1,11) = 0;
       F(2,11) = 0;
       F(3,11) = 0;

       F(4,11) = (q1 + T*(qd1 + dqd1) - (q1 + T*qd1))/dqd1;
       F(5,11) = 0;
       F(6,11) = 0;
       F(7,11) = 0;
       F(8,11) = 0;
       F(9,11) = 0;
       F(10,11) = 0;

      // v = ((qd + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd1;
       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd1;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,11)  = v(0);

       F(12,11)  = v(1);

       F(13,11)  = v(2);

       F(14,11)  = v(3);

       F(15,11)  = v(4);

       F(16,11)  = v(5);

       F(17,11)  = v(6);

     // ROS_INFO_STREAM("F12_old) \n" << F.block(0,11,18,1));


//13th column of the Jacobian matrix(derivative respect to qd2)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2 + dqd2, qd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,12) = 0;
       F(1,12) = 0;
       F(2,12) = 0;
       F(3,12) = 0;

       F(4,12) = 0;
       F(5,12) = (q2 + T*(qd2 + dqd2) - (q2 + T*qd2))/dqd2;
       F(6,12) = 0;
       F(7,12) = 0;
       F(8,12) = 0;
       F(9,12) = 0;
       F(10,12) = 0;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,12)  = v(0);

       F(12,12)  = v(1);

       F(13,12)  = v(2);

       F(14,12)  = v(3);

       F(15,12)  = v(4);

       F(16,12)  = v(5);

       F(17,12)  = v(6);

       //ROS_INFO_STREAM("F13_old) \n" << F.block(0,12,18,1));


//14th column of the Jacobian matrix(derivative respect to qd3)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2, qd3 + dqd3, qd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,13) = 0;
       F(1,13) = 0;
       F(2,13) = 0;
       F(3,13) = 0;

       F(4,13) = 0;
       F(5,13) = 0;
       F(6,13) = (q3 + T*(qd3 + dqd3) - (q3 + T*qd3))/dqd3;
       F(7,13) = 0;
       F(8,13) = 0;
       F(9,13) = 0;
       F(10,13) = 0;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,13)  = v(0);

       F(12,13)  = v(1);

       F(13,13)  = v(2);

       F(14,13)  = v(3);

       F(15,13)  = v(4);

       F(16,13)  = v(5);

       F(17,13)  = v(6);

       //ROS_INFO_STREAM("F14_old) \n" << F.block(0,13,18,1));


//15th column of the Jacobian matrix(derivative respect to qd4)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2, qd3, qd4 + dqd4, qd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,14) = 0;
       F(1,14) = 0;
       F(2,14) = 0;
       F(3,14) = 0;

       F(4,14) = 0;
       F(5,14) = 0;
       F(6,14) = 0;
       F(7,14) = (q4 + T*(qd4 + dqd4) - (q4 + T*qd4))/dqd4;
       F(8,14) = 0;
       F(9,14) = 0;
       F(10,14) = 0;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,14)  = v(0);

       F(12,14)  = v(1);

       F(13,14)  = v(2);

       F(14,14)  = v(3);

       F(15,14)  = v(4);

       F(16,14)  = v(5);

       F(17,14)  = v(6);

      //ROS_INFO_STREAM("F15_old) \n" << F.block(0,14,18,1));

//16th column of the Jacobian matrix(derivative respect to qd5)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5 + dqd5, qd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,15) = 0;
       F(1,15) = 0;
       F(2,15) = 0;
       F(3,15) = 0;

       F(4,15) = 0;
       F(5,15) = 0;
       F(6,15) = 0;
       F(7,15) = 0;
       F(8,15) = (q5 + T*(qd5 + dqd5) - (q5 + T*qd5))/dqd5;
       F(9,15) = 0;
       F(10,15) = 0;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,15)  = v(0);

       F(12,15)  = v(1);

       F(13,15)  = v(2);

       F(14,15)  = v(3);

       F(15,15)  = v(4);

       F(16,15)  = v(5);

       F(17,15)  = v(6);

       //ROS_INFO_STREAM("F16_old) \n" << F.block(0,15,18,1));

//17th column of the Jacobian matrix(derivative respect to qd6)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6 + dqd6, qd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,16) = 0;
       F(1,16) = 0;
       F(2,16) = 0;
       F(3,16) = 0;

       F(4,16) = 0;
       F(5,16) = 0;
       F(6,16) = 0;
       F(7,16) = 0;
       F(8,16) = 0;
       F(9,16) = (q6 + T*(qd6 + dqd6) - (q6 + T*qd6))/dqd6;
       F(10,16) = 0;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,16)  = v(0);

       F(12,16)  = v(1);

       F(13,16)  = v(2);

       F(14,16)  = v(3);

       F(15,16)  = v(4);

       F(16,16)  = v(5);

       F(17,16)  = v(6);

      //ROS_INFO_STREAM("F17_old) \n" << F.block(0,16,18,1));


//18th column of the Jacobian matrix(derivative respect to qd7)

        X_inc << x1, x2 , x3, x4 , q1, q2, q3 ,q4, q5, q6, q7, qd1, qd2, qd3, qd4, qd5, qd6, qd7 + dqd7;

       q_inc = X_inc.block(4,0,7,1);
       qd_inc = X_inc.block(11,0,7,1);


       q_joint_inc.data = q_inc;
       qd_joint_inc.data = qd_inc;
       //ROS_INFO_STREAM("v \n" << q_inc);

       da.B_calc_inc(q_joint_inc);
       da.G_calc_inc(q_joint_inc);
       da.C_calc_inc(q_joint_inc,qd_joint_inc);


       v << 0, 0, 0, 0, 0, 0, 0;

       //Calcolo tau_m per il vettore con incremento
       comput_tau_mm(X_inc,tau_m_inc, da);

       //Ricalcolo quindi tau_m
       comput_tau_mm(X,tau_m, da);



       F(0,17) = 0;
       F(1,17) = 0;
       F(2,17) = 0;
       F(3,17) = 0;

       F(4,17) = 0;
       F(5,17) = 0;
       F(6,17) = 0;
       F(7,17) = 0;
       F(8,17) = 0;
       F(9,17) = 0;
       F(10,17) = (q7 + T*(qd7 + dqd7) - (q7 + T*qd7))/dqd7;

       v = ((qd_joint_inc.data + T*da.B_mat_inc.data.inverse()*(tau + tau_m_inc - da.C_vect_inc.data - da.G_vectt_inc.data)) - (qd + T*da.B_mat.data.inverse()*(tau + tau_m - da.C_vect.data - da.G_vectt.data)))/dqd2;

       //ROS_INFO_STREAM("v \n" << da.B_mat_inc.data.inverse());

       F(11,17)  = v(0);

       F(12,17)  = v(1);

       F(13,17)  = v(2);

       F(14,17)  = v(3);

       F(15,17)  = v(4);

       F(16,17)  = v(5);

       F(17,17)  = v(6);

       //ROS_INFO_STREAM("F18_old) \n" << F.block(0,17,18,1));

       return;

       
   }

*/

