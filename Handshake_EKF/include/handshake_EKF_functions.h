#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <math.h>
//#include "filter/ClassArm.h"


using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using namespace std;


extern double PI;
extern double m1;
extern double m2;
extern double m3;
extern double l1;
extern double l2;
extern double l3;
extern double Iz1;
extern double Iz2;
extern double Iz3;
extern double g;
extern double Kel;
extern double Ktq;
extern double Bsm;
extern double Btq;
extern double T;
extern double Kr;
extern double Dr;


// void comput_tau_m(VectorXd& x, VectorXd& tau_m, MatrixXd& Jac, KDL::Frame& X_ee);
// void comput_tau_mm(VectorXd& x, VectorXd& tau_m, DualArms& da);

// void B_matrix(MatrixXd& B, MatrixXd& B_inv, VectorXd& x);

// void C_matrix(MatrixXd& C, VectorXd& x);

// void G_matrix(VectorXd& G, VectorXd& x);

void VectorStateInit(VectorXd& VectState);

void PInit( MatrixXd& P_mat);

void QInit(MatrixXd& Q_mat);

void RInit(MatrixXd& R_mat);

void F_matrix(MatrixXd& F, VectorXd& X, double m);
// void F_matrixx(MatrixXd& F, VectorXd& X, VectorXd& dX, DualArms& da, MatrixXd& B_matrix, MatrixXd& B_matrix_inv, VectorXd& Cqd_vector,VectorXd& G_vector );

Eigen::Matrix<double,3,3> quat_rot(double x, double y, double z, double w);
Eigen::Matrix<double,6,1> World2Base(Eigen::Matrix<double,3,1> pos_w, Eigen::Matrix<double,3,1> or_w );

// int check(VectorXd& goal_pos, VectorXd& goal_or, DualArms& da);

#endif
