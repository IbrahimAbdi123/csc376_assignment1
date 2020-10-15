#include <pandarobotmodel.h>
#include <iostream>
#include <math.h>
#define M_PI 3.14159265358979323846
/// \brief Constructor
PandaRobotModel::PandaRobotModel()
{
	
}

PandaRobotModel::~PandaRobotModel()
{	
	
}

Eigen::Matrix4d getTransformation(float alphai, float ai, float di, float thetai)
{
    Eigen::Matrix4d T;
    T << cos(thetai),-1*sin(thetai),0,ai,
         sin(thetai)*cos(alphai),cos(thetai)*cos(alphai),-(sin(alphai)),-(di*sin(alphai)),
         sin(thetai)*sin(alphai),cos(thetai)*sin(alphai),cos(alphai),di*cos(alphai),
         0,0,0,1;
    return T;
}

Eigen::Matrix4d PandaRobotModel::forwardKinematicsDH(Eigen::MatrixXd q)
{	
    //std::cout << q << std::endl << std::endl;
    Eigen::Matrix4d Ts_1 = getTransformation(0,0,0.3330,q(0));
    Eigen::Matrix4d T1_2 = getTransformation(-(M_PI/2),0,0,q(1));
    Eigen::Matrix4d T2_3 = getTransformation((M_PI/2),0,0.3160,q(2));
    Eigen::Matrix4d T3_4 = getTransformation((M_PI/2),0.0825,0,q(3));
    Eigen::Matrix4d T4_5 = getTransformation(-(M_PI/2),-0.0825,0.3840,q(4));
    Eigen::Matrix4d T5_6 = getTransformation((M_PI/2),0,0,q(5));
    Eigen::Matrix4d T6_7 = getTransformation((M_PI/2),0.0880,0,q(6));
    Eigen::Matrix4d T7_b = getTransformation(0,0,0.1070,0);

    Eigen::Matrix4d links = (((((T1_2 * T2_3) * T3_4) * T4_5) * T5_6) * T6_7);
    
    Eigen::Matrix4d res = (Ts_1 * links) * T7_b;


    return res;
}

