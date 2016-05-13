
#include "ImpedanceController.h"
#include <iostream>

using namespace std;

ImpedanceController::ImpedanceController() {
  bJointSpaceDamping = false;
}

void ImpedanceController::Resize_e() {
  int nJoints = kdl_chain.getNrOfJoints();


  if (!nJoints) {
    cout << "Impedance controller: kdl chain is empty !" << endl;
    exit(0);
  }

  e_jacobian.resize(6, nJoints);
  e_jacobianT.resize(nJoints, 6);
  e_control_torque.resize(nJoints, 1);

  e_err.resize(6);
  e_err_dot.resize(6);
  e_pos6_dot.resize(6);

  e_jnt_pos_prev.resize(nJoints);
  e_jnt_pos_prev.setZero();

  e_jnt_pos_dot.resize(nJoints);

  e_temp6x1.resize(6);

  e_Kp.setZero();
  e_Kd.setZero();

  e_jointSpaceDamping.resize(nJoints);
  e_jointSpaceDamping.setZero();
}

void ImpedanceController::SetStiffness_e(double StiffnessScalar) {
  for (int i = 0; i < 6; i++) e_Kp(i, i) = StiffnessScalar;
}

void ImpedanceController::SetStiffnessPosition_e(Eigen::Matrix3d StiffnessMatrix) {
  e_Kp.block(0, 0, 3, 3) = StiffnessMatrix;
}

void ImpedanceController::SetStiffnessPosition_e(double StiffnessScalar) {
  for (int i = 0; i < 3; i++) e_Kp(i, i) = StiffnessScalar;
}

void ImpedanceController::SetStiffnessOrientation_e(Eigen::Matrix3d StiffnessMatrix) {
  e_Kp.block(3, 3, 3, 3) = StiffnessMatrix;
}

void ImpedanceController::SetStiffnessOrientation_e(double StiffnessScalar) {
  for (int i = 3; i < 6; i++) e_Kp(i, i) = StiffnessScalar;
}

void ImpedanceController::SetDamping_e(double DampingScalar) {
  for (int i = 0; i < 6; i++) e_Kd(i, i) = DampingScalar;
}

void ImpedanceController::SetDampingPosition_e(double DampingScalar) {
  for (int i = 0; i < 3; i++) e_Kd(i, i) = DampingScalar;
}

void ImpedanceController::SetDampingOrientation_e(double DampingScalar) {
  for (int i = 3; i < 6; i++) e_Kd(i, i) = DampingScalar;
}

void ImpedanceController::EnableJointSpaceDamping(bool en)
{
  bJointSpaceDamping = en;
}

void ImpedanceController::SetJointSpaceDamping_e(Eigen::VectorXd JSDamp)
{
  if ( JSDamp.rows() == e_jointSpaceDamping.rows() ) e_jointSpaceDamping = JSDamp;
}

void ImpedanceController::SetJointSpaceDamping_e(int ind, double damp)
{
  e_jointSpaceDamping(ind) = damp;
}

void ImpedanceController::SetJointSpaceDamping_e(double damp)
{
  e_jointSpaceDamping.setConstant(damp);
}

void ImpedanceController::EnableNullspaceControl(bool en)
{
  bNullspaceControl = en;
}

void ImpedanceController::SetTarget_e(const Eigen::Vector3d TargetPosition, const Eigen::Matrix3d TargetOrient) {
  e_des_pos    = TargetPosition;
  e_des_orient = TargetOrient;
}

void ImpedanceController::Update_e(const KDL::JntArray _jnts) {
  /// a) get Jacobian
  KDL::Jacobian kld_jac;
  KDL::Frame    kdlf_frame;

  kld_jac.resize( kdl_chain.getNrOfJoints() );

  /// Prepare solvers
  KDL::ChainJntToJacSolver kdlcjsolver_endeff        = KDL::ChainJntToJacSolver(kdl_chain);
  KDL::ChainFkSolverPos_recursive kdlfksolver_endeff = KDL::ChainFkSolverPos_recursive(kdl_chain);

  /// Compute the Jacobian and the Cartesian Position
  kdlcjsolver_endeff.JntToJac(_jnts, kld_jac);
  kdlfksolver_endeff.JntToCart(_jnts, kdlf_frame);

  e_jacobian  = kld_jac.data;
  e_jacobianT = e_jacobian.transpose();

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > e_mapM(kdlf_frame.M.data);
  e_curr_orient = e_mapM;

  Eigen::Map<Eigen::Vector3d> e_mapV(kdlf_frame.p.data);
  e_curr_pos = e_mapV;

  e_temp3x3    = e_curr_orient.transpose();
  e_err_orient = e_des_orient * e_temp3x3;

  Eigen::AngleAxisd e_angleaxis(e_err_orient);
  e_err_axis = e_angleaxis.axis() * e_angleaxis.angle();

  // update the err vector containig the positional error and the orientation axis error concatenated. This format corresponds to the Jacobian. Alternative that needs some work: use quaternion representation for the orientation part. Then the jacobian needs to be rebuilt.
  for (int i = 0; i < 3; i++) {
    e_err(i)     = e_des_pos(i) - e_curr_pos(i);
    e_err(i + 3) = e_err_axis(i);
  }

  e_jnt_pos_dot = (_jnts.data - e_jnt_pos_prev) / 0.001; // TODO: replace the value by variable

  e_pos6_dot = e_jacobian * e_jnt_pos_dot;               // careful: only damping on velocity ... // x_dot = J*q_dot // is it right ??
  e_err_dot  = -e_pos6_dot;

  // kp and kd
  e_control_wrench  = e_Kp * e_err;
  e_temp6x1         = e_Kd * e_err_dot;
  e_control_wrench += e_temp6x1;


  e_control_torque = e_jacobianT * e_control_wrench;


  // add Joint space damping
  e_joint_damping_torques = e_jointSpaceDamping.cwiseProduct(e_jnt_pos_dot);
//  cerr << "e_joint_damping_torques before: /n " << e_joint_damping_torques << endl;

  for (int i = 0; i < e_joint_damping_torques.rows(); i++) {
    if (e_joint_damping_torques(i) > 50.0) e_joint_damping_torques(i) = 50.0;

    if (e_joint_damping_torques(i) < -50.0) e_joint_damping_torques(i) = -50.0;
  }
//  cerr << "e_joint_damping_torques: /n " << e_joint_damping_torques << endl;
//  cerr << "e_jnt_pos_dot: /n " << e_jnt_pos_dot << endl;

  if (bJointSpaceDamping) e_control_torque -= e_joint_damping_torques;

  e_jnt_pos_prev = _jnts.data;
  e_prev_pos     = e_curr_pos;
  e_prev_des_pos = e_des_pos;
}

void ImpedanceController::InitChain(KDL::Chain _chain) {
  kdl_chain = _chain; // does it copy the whole chain or is it a link ? not sure ...

  Resize_e();
}

void ImpedanceController::GetOutput_e(Eigen::Matrix<double, Eigen::Dynamic, 1>& result) {
  if ( result.rows() == e_control_torque.rows() ) result = e_control_torque;
  else {
    result.resize( e_control_torque.rows() ); // not sure about this one
    result = e_control_torque;
  }
}

void ImpedanceController::GetError_e(Eigen::Matrix<double, Eigen::Dynamic, 1>& error) {
  error = e_err;
}
