#include <pandarobotmodel.h>


	
/// \brief Constructor
PandaRobotModel::PandaRobotModel()
{
	
}

PandaRobotModel::~PandaRobotModel()
{	
	
}

Eigen::Matrix4d PandaRobotModel::forwardKinematicsDH(Eigen::MatrixXd q)
{	

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();


    return T;
}

