#define _USE_MATH_DEFINES
#define MAXBUFFSIZE ((int) 1e6)

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iomanip>

using namespace UNITREE_LEGGED_SDK;
using namespace std;

using Eigen::Matrix;
using Eigen::Vector;
using Eigen::seqN;
using std::cos; using std::sin;

typedef Matrix<float, 4, 4> Matrix4d;
typedef Matrix<float, 6, 3> Matrix63d;
typedef Matrix<float, 6, 4> Matrix64d;
typedef Matrix<float, 3, 4> Matrix34d;
typedef Matrix<float, 3, 3> Matrix33d;
typedef Vector<float, 3> Vector3d;
typedef Vector<float, 4> Vector4d;

Matrix4d DH(const string leg, const int i, Matrix64d d, Matrix64d theta, Matrix64d a, Matrix64d alpha) {

    Matrix64d cos_theta = theta.array().cos(); 
    Matrix64d sin_theta = theta.array().sin();
    Matrix64d cos_alpha = alpha.array().cos();
    Matrix64d sin_alpha = alpha.array().sin();

    // instead of pasing in the leg string probably should just pass in j directly
    int j;
    if (leg == "FR") {
        j = 0; 
    } else if (leg == "FL") {
        j = 1;
    } else if (leg == "RR") {
        j = 2;
    } else if (leg == "RL") {
        j = 3;
    }

    Matrix4d HT;
    HT << cos_theta.block(i,j,1,1), -sin_theta.block(i,j,1,1)*cos_alpha.block(i,j,1,1), sin_theta.block(i,j,1,1)*sin_alpha.block(i,j,1,1), a.block(i,j,1,1)*cos_theta.block(i,j,1,1),
        sin_theta.block(i,j,1,1), cos_theta.block(i,j,1,1)*cos_alpha.block(i,j,1,1), -cos_theta.block(i,j,1,1)*sin_alpha.block(i,j,1,1), a.block(i,j,1,1)*sin_theta.block(i,j,1,1),
        0, sin_alpha.block(i,j,1,1), cos_alpha.block(i,j,1,1), d.block(i,j,1,1),
        0, 0, 0, 1;

    return HT;
}


Matrix63d Jacobian(Matrix4d TAB, Matrix4d TB0, Matrix4d T01, Matrix4d T12, Matrix4d T23, Matrix4d T3E) {

    Vector3d p0, p1, p2, p3, pE, z1, z2, z3, zE; 

    Matrix4d TA0; Matrix4d TA1; Matrix4d TA2; Matrix4d TA3; Matrix4d TAE;
    TA0 = TAB*TB0; TA1 = TA0*T01; TA2 = TA1*T12; TA3 = TA2*T23; TAE = TA3*T3E;
 
    p1 = TA0.block(0,3,3,1); p2 = TA2.block(0,3,3,1); p3 = TA3.block(0,3,3,1); pE = TAE.block(0,3,3,1);
    // cout << p1 << endl << p2 << endl << p3 << endl << pE << endl;
    z1 << TA0.block(0,2,3,1);
    z2 << TA2.block(0,2,3,1);
    z3 << TA3.block(0,2,3,1);
    zE << TAE.block(0,2,3,1);
    // cout << z1 << endl << z2 << endl << z3 << endl << zE << endl;

    Matrix33d P; Matrix34d Z;
    P << p1, p2, p3;  
    Z << z1, z2, z3, zE;

    Matrix63d J;  Vector3d difference; Vector3d cross; 
    for (int i = 0; i < 3; i++) {
        difference = pE.block(0,0,3,1) - P.col(i);
        cross = Z.col(i).cross(difference);
        J.col(i).topRows<3>() = cross; 
        J.col(i).bottomRows<3>() = Z.col(i);
    }
    return J;
}

Eigen::MatrixXd readMatrix(const char *filename) {
    int cols = 0, rows = 0;
    double buff[MAXBUFFSIZE];

    // Read values from file into buffer
    ifstream infile;
    infile.open(filename);
    if (!infile.is_open()) {
        cout << "Error opening file!" << endl;
    }

    while (! infile.eof()) {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof()) {
            stream >> buff[cols*rows+temp_cols++];
        }

        if (temp_cols == 0) {
            continue;
        }
        if (cols == 0) {
            cols = temp_cols;
        }
        rows++;
    }
    infile.close();
    rows--;

    // Populate matrix with the values
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result(i,j) = buff[cols*i+j];
        }
    }
    return result;
}


int main(void) {

    double b_width = 0.0935; double b_length = 0.3762;        // base width & length
    double l0 = 0.08; double l1 = 0.213; double l2 = 0.213;   // leg lengths

    // DH Parameters 
    // Columns: Front Right, Front Left, Rear Right, Rear Left 
    // Rows: Link A-B, B-0, 0-1, 1-2, 2-3, 3-E
    Matrix64d d; Matrix64d theta; Matrix64d a; Matrix64d alpha;
    d << 0, 0, 0, 0,
        b_length/2, b_length/2, -b_length/2, -b_length/2,
        0, 0, 0, 0,
        0, 0, 0, 0, 
        0, 0, 0, 0,
        0, 0, 0, 0;
    theta << M_PI_2, M_PI_2, M_PI_2, M_PI_2, 
        0, 0, 0, 0, 
        0, 0, 0, 0,
        -M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, 
        0, 0, 0, 0, 
        0, 0, 0, 0;
    a << -b_width/2, b_width/2, -b_width/2, b_width/2, 
        0, 0, 0, 0,
        -l0, l0, -l0, l0,
        0, 0, 0, 0,
        l1, l1, l1, l1,
        l2, l2, l2, l2;
    alpha << M_PI_2, M_PI_2, M_PI_2, M_PI_2,
        0, 0, 0, 0,
        0, 0, 0, 0,
        -M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2,
        0, 0, 0, 0,
        0, 0, 0, 0;  

    Eigen::MatrixXd joint_tau = readMatrix("walking_joint_tau.txt");     // load joint torques
    Eigen::MatrixXd joint_q = readMatrix("walking_joint_q.txt");         // load joint positions

    Matrix4d FR_AB; Matrix4d FR_B0; Matrix4d FR_01; Matrix4d FR_12; Matrix4d FR_23; Matrix4d FR_3E; 
    Matrix4d FL_AB; Matrix4d FL_B0; Matrix4d FL_01; Matrix4d FL_12; Matrix4d FL_23; Matrix4d FL_3E; 
    Matrix4d RR_AB; Matrix4d RR_B0; Matrix4d RR_01; Matrix4d RR_12; Matrix4d RR_23; Matrix4d RR_3E; 
    Matrix4d RL_AB; Matrix4d RL_B0; Matrix4d RL_01; Matrix4d RL_12; Matrix4d RL_23; Matrix4d RL_3E; 

    // Front Right Leg
    Vector3d theta_jointq_FR; Matrix64d angles_FR = theta;
    theta_jointq_FR << theta.block(2,0,1,1) + joint_q.block(56,0,1,1).cast<float>(),
                    theta.block(4,0,1,1) + joint_q.block(56,1,1,1).cast<float>(),
                    theta.block(5,0,1,1) + joint_q.block(56,2,1,1).cast<float>();

    angles_FR.block(2,0,1,1) = theta_jointq_FR.block(0,0,1,1);
    angles_FR.block(4,0,1,1) = theta_jointq_FR.block(1,0,1,1); 
    angles_FR.block(5,0,1,1) = theta_jointq_FR.block(2,0,1,1);

    FR_AB = DH("FR",0,d,angles_FR,a,alpha); FR_B0 = DH("FR",1,d,angles_FR,a,alpha); FR_01 = DH("FR",2,d,angles_FR,a,alpha); 
    FR_12 = DH("FR",3,d,angles_FR,a,alpha); FR_23 = DH("FR",4,d,angles_FR,a,alpha); FR_3E = DH("FR",5,d,angles_FR,a,alpha); 
    Matrix4d FR_A1 = FR_AB*FR_01; Matrix4d FR_A2 = FR_AB*FR_B0*FR_01*FR_12;
    Matrix4d FR_A3 = FR_AB*FR_B0*FR_01*FR_12*FR_23;
    Matrix4d FR_AE = FR_AB*FR_B0*FR_01*FR_12*FR_23*FR_3E;

    // Front Left Leg
    Vector3d theta_jointq_FL; Matrix64d angles_FL = theta;
    theta_jointq_FL << theta.block(2,0,1,1) + joint_q.block(57,0,1,1).cast<float>(),
                theta.block(4,0,1,1) + joint_q.block(57,1,1,1).cast<float>(),
                theta.block(5,0,1,1) + joint_q.block(57,2,1,1).cast<float>();

    angles_FL.block(2,1,1,1) = theta_jointq_FL.block(0,0,1,1);
    angles_FL.block(4,1,1,1) = theta_jointq_FL.block(1,0,1,1); 
    angles_FL.block(5,1,1,1) = theta_jointq_FL.block(2,0,1,1);

    FL_AB = DH("FL",0,d,angles_FL,a,alpha); FL_B0 = DH("FL",1,d,angles_FL,a,alpha); FL_01 = DH("FL",2,d,angles_FL,a,alpha); 
    FL_12 = DH("FL",3,d,angles_FL,a,alpha); FL_23 = DH("FL",4,d,angles_FL,a,alpha); FL_3E = DH("FL",5,d,angles_FL,a,alpha); 
    Matrix4d FL_A1 = FL_AB*FL_B0*FL_01; Matrix4d FL_A2 = FL_AB*FL_B0*FL_01*FL_12;
    Matrix4d FL_A3 = FL_AB*FL_B0*FL_01*FL_12*FL_23;
    Matrix4d FL_AE = FL_AB*FL_B0*FL_01*FL_12*FL_23*FL_3E;

    // Rear Right Leg
    Vector3d theta_jointq_RR; Matrix64d angles_RR = theta;
    theta_jointq_RR << theta.block(2,0,1,1) + joint_q.block(58,0,1,1).cast<float>(),
                theta.block(4,0,1,1) + joint_q.block(58,1,1,1).cast<float>(),
                theta.block(5,0,1,1) + joint_q.block(58,2,1,1).cast<float>();

    angles_RR.block(2,2,1,1) = theta_jointq_RR.block(0,0,1,1);
    angles_RR.block(4,2,1,1) = theta_jointq_RR.block(1,0,1,1); 
    angles_RR.block(5,2,1,1) = theta_jointq_RR.block(2,0,1,1);

    RR_AB = DH("RR",0,d,angles_RR,a,alpha); RR_B0 = DH("RR",1,d,angles_RR,a,alpha); RR_01 = DH("RR",2,d,angles_RR,a,alpha);
    RR_12 = DH("RR",3,d,angles_RR,a,alpha); RR_23 = DH("RR",4,d,angles_RR,a,alpha); RR_3E = DH("RR",5,d,angles_RR,a,alpha); 
    Matrix4d RR_A1 = RR_AB*RR_B0*RR_01; Matrix4d RR_A2 = RR_AB*RR_B0*RR_01*RR_12;
    Matrix4d RR_A3 = RR_AB*RR_B0*RR_01*RR_12*RR_23;
    Matrix4d RR_AE = RR_AB*RR_B0*RR_01*RR_12*RR_23*RR_3E;

    // Rear Left Leg
    Vector3d theta_jointq_RL; Matrix64d angles_RL = theta;
    theta_jointq_RL << theta.block(2,3,1,1) + joint_q.block(59,0,1,1).cast<float>(),
                theta.block(4,3,1,1) + joint_q.block(59,1,1,1).cast<float>(),
                theta.block(5,3,1,1) + joint_q.block(59,2,1,1).cast<float>();

    angles_RL.block(2,3,1,1) = theta_jointq_RL.block(0,0,1,1);
    angles_RL.block(4,3,1,1) = theta_jointq_RL.block(1,0,1,1); 
    angles_RL.block(5,3,1,1) = theta_jointq_RL.block(2,0,1,1);

    RL_AB = DH("RL",0,d,angles_RL,a,alpha); RL_B0 = DH("RL",1,d,angles_RL,a,alpha); RL_01 = DH("RL",2,d,angles_RL,a,alpha);
    RL_12 = DH("RL",3,d,angles_RL,a,alpha); RL_23 = DH("RL",4,d,angles_RL,a,alpha); RL_3E = DH("RL",5,d,angles_RL,a,alpha); 
    Matrix4d RL_A1 = RL_AB*RL_B0*RL_01; Matrix4d RL_B2 = RL_AB*RL_B0*RL_01*RL_12;
    Matrix4d RL_A3 = RL_AB*RL_B0*RL_01*RL_12*RL_23;
    Matrix4d RL_AE = RL_AB*RL_B0*RL_01*RL_12*RL_23*RL_3E;

    // Jacobian Calculation
    Matrix63d FR_J = Jacobian(FR_AB,FR_B0,FR_01,FR_12,FR_23,FR_3E); 
    Matrix63d FL_J = Jacobian(FL_AB,FL_B0,FL_01,FL_12,FL_23,FL_3E); 
    Matrix63d RR_J = Jacobian(RR_AB,RR_B0,RR_01,RR_12,RR_23,RR_3E); 
    Matrix63d RL_J = Jacobian(RL_AB,RL_B0,RL_01,RL_12,RL_23,RL_3E);

    Matrix33d FR_JTI = FR_J.block(0,0,3,3).transpose().inverse();
    Vector3d X = FR_JTI * joint_tau.block(56,0,1,3).transpose().cast<float>();
    Eigen::MatrixXd FR_Forces = X.cast<double>();

    Matrix33d FL_JTI = FL_J.block(0,0,3,3).transpose().inverse();
    Vector3d Y = FL_JTI * joint_tau.block(57,0,1,3).transpose().cast<float>();
    Eigen::MatrixXd FL_Forces = Y.cast<double>();

    Matrix33d RR_JTI = RR_J.block(0,0,3,3).transpose().inverse();
    Vector3d Z = RR_JTI * joint_tau.block(58,0,1,3).transpose().cast<float>();
    Eigen::MatrixXd RR_Forces = Z.cast<double>();

    Matrix33d RL_JTI = RL_J.block(0,0,3,3).transpose().inverse();
    Vector3d A = RL_JTI * joint_tau.block(59,0,1,3).transpose().cast<float>();
    Eigen::MatrixXd RL_Forces = A.cast<double>();

    // End-Effector Force Calculation
    Vector4d total_foot_force;
    total_foot_force(0,0) = FR_Forces.sum();
    total_foot_force(1,0) = FL_Forces.sum();
    total_foot_force(2,0) = RR_Forces.sum();
    total_foot_force(3,0) = RL_Forces.sum();

    cout << "The total force [N] exerted on each foot: " << endl << total_foot_force <<  endl;
    cout << "The total weight: " << total_foot_force.sum() / 9.81 << " [kg]" << endl;

    return 0;
}