
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

#include "Eigen/Eigen"
#include "Eigen/Geometry"


class ImpedanceController {
public:

  ImpedanceController();


  void         Init_e();

  void         InitChain(KDL::Chain);


  void         SetStiffness_e(double StiffnessScalar);
  void         SetStiffnessPosition_e(double StiffnessScalar);
  void         SetStiffnessPosition_e(Eigen::Matrix3d StiffnessMatrix);
  void         SetStiffnessOrientation_e(Eigen::Matrix3d StiffnessMatrix);
  void         SetStiffnessOrientation_e(double StiffnessScalar);
  void         SetDamping_e(double DampingScalar);
  void         SetDampingPosition_e(double DampingScalar);
  void         SetDampingOrientation_e(double DampingScalar);

  void         EnableJointSpaceDamping(bool en);

  void         SetJointSpaceDamping_e(Eigen::VectorXd JSDamp);
  void         SetJointSpaceDamping_e(double JSDamp);
  void         SetJointSpaceDamping_e(int    ind,
                                      double damp);

  void         EnableNullspaceControl(bool en);

  void         SetTarget_e(const Eigen::Vector3d TargetPosition,
                           const Eigen::Matrix3d TargetOrient);

  virtual void Update_e(const KDL::JntArray _jnts);

  void         GetOutput_e(Eigen::Matrix<double, Eigen::Dynamic, 1>& result);

  void         GetError_e(Eigen::Matrix<double, Eigen::Dynamic, 1>& error);

  virtual void Resize_e();

protected:

  bool bJointSpaceDamping;

  bool bNullspaceControl;


  KDL::Chain kdl_chain;
  Eigen::Vector3d e_curr_pos, e_des_pos, e_err_axis, e_curr_pos_dot, e_prev_pos, e_prev_des_pos, e_des_pos_dot;
  Eigen::Matrix3d e_curr_orient, e_des_orient, e_temp3x3, e_err_orient;
  Eigen::Matrix<double, 6, 6> e_Kp, e_Kd;
  Eigen::Matrix<double, Eigen::Dynamic, 1> e_control_wrench, e_control_torque, e_err, e_pos6_dot, e_jnt_pos_prev, e_jnt_pos_dot, e_temp6x1, e_err_dot, e_jointSpaceDamping;
  Eigen::VectorXd e_joint_damping_torques;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> e_jacobian, e_jacobianT;
};

#endif // ifndef IMPEDANCE_CONTROLLER_H
