
#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H



#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>


#include "MathLib/MathLib.h"
#include "RobotLib/Robot.h"
#include "RobotLib/RobotTools.h"
#include "RobotLib/KinematicChain.h"
#include "StdTools/XmlTree.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
//typedef Eigen::Matrix<double, 6,1> Vector6d;

//#include <kdl_parser/kdl_parser.hpp> // read kdl trees from urdf files/parameters, needs to be loaded before mathlib because PI config



//#ifdef USE_MATHLIB_NAMESPACE
//namespace MathLib {
//#endif

  class ImpedanceController {

  public :
    ImpedanceController();
    

//    void SetRobot(Robot* pRob, string targetLink="TOOLPLATE");
    void Init(pXmlTree tree = NULL);
    void Init_e();

    void InitChain(KDL::Chain);


    //    void SetRobot(Robot* pRob);

    //    virtual void Init(pXmlTree tree = NULL);



//    void SetStiffness( Matrix StiffnessMatrix);
//    void SetStiffness( Vector StiffnessVector);
    void SetStiffness( double StiffnessScalar);
    void SetStiffness_e( double StiffnessScalar);
//    void SetStiffnessPosition( Vector3 StiffnessVector);
//    void SetStiffnessPosition( Matrix StiffnessVector);
    void SetStiffnessPosition( double StiffnessScalar);
    void SetStiffnessPosition_e( double StiffnessScalar);
    void SetStiffnessPosition_e( Eigen::Matrix3d StiffnessMatrix);
//    void SetStiffnessOrientation( Vector3 StiffnessVector);
    void SetStiffnessOrientation( double StiffnessScalar);
    void SetStiffnessOrientation_e( Eigen::Matrix3d StiffnessMatrix);
    void SetStiffnessOrientation_e( double StiffnessScalar);

//    void SetDamping( Matrix DampingMatrix);
//    void SetDamping( Vector DampingVector);
    void SetDamping( double DampingScalar);
    void SetDamping_e( double DampingScalar);
//    void SetDampingPosition( Vector3 DampingVector);
    void SetDampingPosition( double DampingScalar);
    void SetDampingPosition_e( double DampingScalar);
//    void SetDampingOrientation( Vector3 DampingVector);
    void SetDampingOrientation( double DampingScalar);
    void SetDampingOrientation_e( double DampingScalar);

    void EnableJointSpaceDamping(bool en);
//    void SetJointSpaceDamping( Vector JSDamp);
    void SetJointSpaceDamping_e(int ind, REALTYPE damp);
    void SetJointSpaceDamping_e(Eigen::VectorXd JSDamp);


    void EnableNullspaceControl(bool en);
//    void SetNullspaceJointTarget(const Vector & target);
//    void SetNullspaceStiffness(const Vector & stiffness);

//    void SetExtraWrench(const Vector &);



    void SetTarget(const Vector3 TargetPosition, const Matrix3 TargetOrient);
    void SetTarget_e(const Eigen::Vector3d TargetPosition, const Eigen::Matrix3d TargetOrient);

//    virtual void Update();
    virtual void Update_e(const KDL::JntArray _jnts);

    void GetOutput(Vector &);
    void GetOutput_e(Eigen::Matrix<double,Eigen::Dynamic,1> &result);

    void GetError_e(Eigen::Matrix<double,Eigen::Dynamic,1> &error);

//    void AddOutput(Vector &);
    virtual void Resize();
    virtual void Resize_e();

  protected :
    Robot* pRobot;
    KinematicChain KinChain;
    RevoluteJointSensorGroup Encoders;
    
    Vector err, err_dot;
    Matrix Kd,Kp;
    Vector3 currPos,desPos,errAxis,errAxis_dot;
    Matrix3 currOrient,desOrient,errOrient,temp3x3;
    Vector ControlWrench,temp6x1,ControlTorque,ExtraWrench;
    bool bHaveRobot;
    bool bJointSpaceDamping;
    int mLinkIndex;
    
    IndicesVector indices;
    Matrix Jacobian,JacobianT,SubJacobian,NullspaceProj,Slask6x6,Slask6x6_2;
    Vector jointSpaceDamping,temp7x1,NullspaceBasis,NullspaceStiffness,JointTarget,NullspaceTorques,NullspaceTorquesAP;
    bool bNullspaceControl;


    ///
    KDL::Chain kdl_chain;
    Eigen::Vector3d e_curr_pos, e_des_pos, e_err_axis,e_curr_pos_dot, e_prev_pos, e_prev_des_pos,e_des_pos_dot;
    Eigen::Matrix3d e_curr_orient, e_des_orient, e_temp3x3, e_err_orient;
    Eigen::Matrix<double,6,6> e_Kp,e_Kd;
    Eigen::Matrix<double,Eigen::Dynamic,1> e_control_wrench,e_control_torque, e_err, e_pos6_dot, e_jnt_pos_prev,e_jnt_pos_dot, e_temp6x1, e_err_dot, e_jointSpaceDamping;
    Eigen::VectorXd e_joint_damping_torques;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> e_jacobian,e_jacobianT;


  };

//#ifdef USE_MATHLIB_NAMESPACE
//}
//#endif
#endif
