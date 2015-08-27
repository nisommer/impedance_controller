
#include "ImpedanceController.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

ImpedanceController::ImpedanceController(){
  pRobot=NULL;
  bHaveRobot = false;

  bJointSpaceDamping=false;
}

void ImpedanceController::Resize(){
  int nJoints = pRobot->GetDOFCount();
  Kd.Resize(6,6);
  Kp.Resize(6,6);
  err.Resize(6);
  err_dot.Resize(6);
  ControlWrench.Resize(6);
  temp6x1.Resize(6,1);
  ExtraWrench.Resize(6);
  Jacobian.Resize(6,nJoints);
  JacobianT.Resize(nJoints,6);
  ControlTorque.Resize(nJoints);
  jointSpaceDamping.Resize(nJoints);
  jointSpaceDamping.Zero();
  temp7x1.Resize(nJoints);


  SubJacobian.Resize(6,6);
  NullspaceProj.Resize(nJoints,nJoints);
  NullspaceBasis.Resize(nJoints);
  NullspaceStiffness.Resize(nJoints);
  indices.resize(6);
  bNullspaceControl = false;
  Slask6x6.Resize(6,6);
  Slask6x6_2.Resize(6,6);
  JointTarget.Resize(nJoints);
  NullspaceTorques.Resize(nJoints);
  NullspaceTorquesAP.Resize(nJoints);
  //    e_jointSpaceDamping.resize(nJoints);
  //    e_jointSpaceDamping.setZero();
}


void ImpedanceController::Resize_e(){
  //  int nJoints = pRobot->GetDOFCount();
  int nJoints = kdl_chain.getNrOfJoints();
  //  Kd.Resize(6,6);
  //  Kp.Resize(6,6);
  //      err.Resize(6);
  //      err_dot.Resize(6);
  //  ControlWrench.Resize(6);
  //  temp6x1.Resize(6,1);
  //  ExtraWrench.Resize(6);
  Jacobian.Resize(6,nJoints);
  JacobianT.Resize(nJoints,6);
  ControlTorque.Resize(nJoints);

  if (!nJoints){
    cout << "Impedance controller: kdl chain is empty !" << endl;
    exit(0);
  }

  e_jacobian.resize(6,nJoints);
  e_jacobianT.resize(nJoints,6);
  e_control_torque.resize(nJoints,1);

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
  //    bJointSpaceDamping
  //            jointSpaceDamping




  //  jointSpaceDamping.Resize(nJoints);
  //  jointSpaceDamping.Zero();
  //  temp7x1.Resize(nJoints);




  //  SubJacobian.Resize(6,6);
  //  NullspaceProj.Resize(nJoints,nJoints);
  //  NullspaceBasis.Resize(nJoints);
  //  NullspaceStiffness.Resize(nJoints);
  //  indices.resize(6);
  //  bNullspaceControl = false;
  //  Slask6x6.Resize(6,6);
  //  Slask6x6_2.Resize(6,6);
  //  JointTarget.Resize(nJoints);
  //  NullspaceTorques.Resize(nJoints);
  //  NullspaceTorquesAP.Resize(nJoints);
}


//void ImpedanceController::SetStiffness(Matrix StiffnessMatrix){
//  Kp=StiffnessMatrix;
//}

//void ImpedanceController::SetStiffness(Vector StiffnessVector){
//    if (StiffnessVector.Size() == 6){
//        for (int i=0;i<6;i++)
//            Kp(i,i) = StiffnessVector(i);
//    }else{
//        cout<<"wrong dimension!"<<endl;
//    }
//}

//void ImpedanceController::SetStiffness(double StiffnessScalar){
//    for (int i=0;i<6;i++)
//        Kp(i,i) = StiffnessScalar;
//}

void ImpedanceController::SetStiffness_e(double StiffnessScalar){
  for (int i=0;i<6;i++)
    e_Kp(i,i) = StiffnessScalar;
}

//void ImpedanceController::SetStiffnessPosition(Vector3 StiffnessVector){
//    for (int i=0;i<3;i++)
//        Kp(i,i) = StiffnessVector(i);
//}

//void ImpedanceController::SetStiffnessPosition(Matrix StiffnessPositionMatrix){
//    for (int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            Kp(i,j) = StiffnessPositionMatrix(i,j);
//        }
//    }
//}

void ImpedanceController::SetStiffnessPosition_e(Eigen::Matrix3d StiffnessMatrix){
  e_Kp.block(0,0,3,3)=StiffnessMatrix;
  //    for (int i=0;i<3;i++){
  //        for(int j=0;j<3;j++){
  //            e_Kp(i,j) = StiffnessMatrix(i,j);
  //        }
  //    }
}

//void ImpedanceController::SetStiffnessPosition(double StiffnessScalar){
//    for (int i=0;i<3;i++)
//        Kp(i,i) = StiffnessScalar;
//}

void ImpedanceController::SetStiffnessPosition_e(double StiffnessScalar){
  for (int i=0;i<3;i++)
    e_Kp(i,i) = StiffnessScalar;

}

//void ImpedanceController::SetStiffnessOrientation(Vector3 StiffnessVector){
//    for (int i=3;i<6;i++)
//        Kp(i,i) = StiffnessVector(i-3);
//}

//void ImpedanceController::SetStiffnessOrientation(Vector3 StiffnessVector){
//    for (int i=3;i<6;i++)
//        Kp(i,i) = StiffnessVector(i-3);
//}

void ImpedanceController::SetStiffnessOrientation_e(Eigen::Matrix3d StiffnessMatrix){
  e_Kp.block(3,3,3,3)=StiffnessMatrix;
}

void ImpedanceController::SetStiffnessOrientation_e(double StiffnessScalar){
  for (int i=3;i<6;i++)
    e_Kp(i,i) = StiffnessScalar;
}

//void ImpedanceController::SetDamping(Matrix DampingMatrix){
//  Kd=DampingMatrix;
//}

//void ImpedanceController::SetDamping(Vector DampingVector){
//    if (DampingVector.Size() == 6){
//        for (int i=0;i<6;i++)
//            Kd(i,i) = DampingVector(i);
//    }else{
//        cout<<"wrong dimension!"<<endl;
//    }
//}

void ImpedanceController::SetDamping(double DampingScalar){
  for (int i=0;i<6;i++)
    Kd(i,i) = DampingScalar;
}

void ImpedanceController::SetDamping_e(double DampingScalar){
  for (int i=0;i<6;i++)
    e_Kd(i,i) = DampingScalar;
}

//void ImpedanceController::SetDampingPosition(Vector3 DampingVector){
//    for (int i=0;i<3;i++)
//        Kd(i,i) = DampingVector(i);
//}

void ImpedanceController::SetDampingPosition(double DampingScalar){
  for (int i=0;i<3;i++)
    Kd(i,i) = DampingScalar;
}

void ImpedanceController::SetDampingPosition_e(double DampingScalar){
  for (int i=0;i<3;i++)
    e_Kd(i,i) = DampingScalar;
}

//void ImpedanceController::SetDampingOrientation(Vector3 DampingVector){
//    for (int i=3;i<6;i++)
//        Kd(i,i) = DampingVector(i-3);
//}

void ImpedanceController::SetDampingOrientation(double DampingScalar){
  for (int i=3;i<6;i++)
    Kd(i,i) = DampingScalar;
}

void ImpedanceController::SetDampingOrientation_e(double DampingScalar){
  for (int i=3;i<6;i++)
    e_Kd(i,i) = DampingScalar;
}

void ImpedanceController::EnableJointSpaceDamping(bool en)
{
  bJointSpaceDamping = en;
}

//void ImpedanceController::SetJointSpaceDamping(Vector JSDamp)
//{
//    if(JSDamp.Size() == jointSpaceDamping.Size())
//        jointSpaceDamping = JSDamp;
//}

void ImpedanceController::SetJointSpaceDamping_e(Eigen::VectorXd JSDamp)
{
  if(JSDamp.rows() == e_jointSpaceDamping.rows())
    e_jointSpaceDamping = JSDamp;
}



void ImpedanceController::SetJointSpaceDamping_e(int ind, REALTYPE damp)
{
  jointSpaceDamping(ind) = damp;
}


void ImpedanceController::EnableNullspaceControl(bool en)
{
  bNullspaceControl = en;
}

//void ImpedanceController::SetNullspaceJointTarget(const Vector &target)
//{
//    JointTarget =target;
//}

//void ImpedanceController::SetNullspaceStiffness(const Vector &stiffness)
//{
//    NullspaceStiffness = stiffness;
//}

//void ImpedanceController::SetExtraWrench(const Vector & userWrench)
//{
//    ExtraWrench = userWrench;
//}

void ImpedanceController::SetTarget(const Vector3 TargetPosition, const Matrix3 TargetOrient){
  desPos = TargetPosition;
  desOrient = TargetOrient;
}

void ImpedanceController::SetTarget_e(const Eigen::Vector3d TargetPosition,const Eigen::Matrix3d TargetOrient){
  e_des_pos= TargetPosition;
  e_des_orient= TargetOrient;
  //  desPos = TargetPosition;
  //  desOrient = TargetOrient;
}

//void ImpedanceController::Update(){
//    KinChain.Update();
//    Encoders.ReadSensors();
//    Jacobian = KinChain.GetJacobian();
//    Jacobian.Transpose(JacobianT);
//    //  Jacobian.Print();
//    // ok
//    //currOrient = pRobot->GetReferenceFrame(pRobot->GetLinksCount()-1,0).GetOrient();
//    currOrient = pRobot->GetReferenceFrame(mLinkIndex,0).GetOrient();
//    //  cout<<"currorient"<<endl;
//    //  cout<<"currorient"<<endl;
//    //  currOrient.Print();
//    // currPos = pRobot->GetReferenceFrame(pRobot->GetLinksCount()-1,0).GetOrigin();
//    currPos = pRobot->GetReferenceFrame(mLinkIndex,0).GetOrigin();

//    temp3x3 = currOrient.Transpose();
//    // desOrient.Mult(temp3x3,errOrient);

//    desOrient.Mult(temp3x3,errOrient);
//    //errOrient = desOrient*(desOrient*(currOrient.Transpose()));
//    errOrient.GetExactRotationAxis(errAxis);

//    /*update the err vector containig the positional error and the orientation axis error concatenated. This format corresponds to the Jacobian. Alternative that needs some work: use quaternion representation for the orientation part. Then the jacobian needs to be rebuilt. */
//    for(int i=0;i<3;i++){
//        err(i) = desPos(i)-currPos(i);
//        err(i+3) = errAxis(i); // or positive ?
//    }
//    //err.Print();
//    //update the cartesian velocity, note that this is not the error vel, it is the actual vel.
//    Jacobian.Mult(Encoders.GetJointVelocities(),err_dot);

//    //ControlWrench = Kp*err - Kd*err_dot;
//    Kp.Mult(err,ControlWrench);
//    Kd.Mult(err_dot,temp6x1);
//    ControlWrench -= temp6x1;
//    ControlWrench += ExtraWrench;

//    //reset the extera wrench to zero !
//    ExtraWrench *=0;

//    JacobianT.Mult(ControlWrench,ControlTorque);
//    //add Joint space damping
//    jointSpaceDamping.PMult(Encoders.GetJointVelocities(),temp7x1);
//    temp7x1 *= -1;
//    ControlTorque += temp7x1;

//    //NOTE NULLSPACE CONTROL IS HIGHLY EXPERIEMENTAL AND DOES NOT REALLY WORK SAFELY. FOR NOW THIS IS DISABLES.
//    //bNullspaceControl = false;
//    //  if(bNullspaceControl){
//    //      /*-------------------------------------------------------
//    //        -----------------------------------------------------------
//    //        -------------------- Nullspace Control-----------
//    //        ---------------------------------------------------------
//    //        -------------------------------------------------------*/
//    //      //since the Jacobian is always one-dimensional, this simple approach may be used to find the basis-vector of this one-dimensional nullspace. Refer to 'Cartesian Impedance Control For Redudant Robots' by  Chrsistian Ott. page 47-48.

//    //      int wi = 0;
//    //      double det = 0;
//    //      //here we set an indices vector for exclusion of one column at a time from the jacoban.
//    //      for(int i=0;i<7;i++){
//    //          wi=0;
//    //          for(int j=0; j<7; j++){
//    //              indices[wi]=j;
//    //              if(j!=i)
//    //                  wi++;
//    //          }
//    ////                cout<<"the indices vector "<< i << indices.size() <<endl;
//    ////                for(int k =0;k<6;k++)
//    ////                    cout<<indices[k]<<" ";
//    ////                cout<<endl;
//    //          Jacobian.GetColumnSpace(indices,SubJacobian);
//    //         // SubJacobian.Print();
//    //          SubJacobian.Inverse(Slask6x6,&det,R_ONE,&Slask6x6_2);
//    //        //  cout<<SubJacobian.IsInverseOk()<<endl;
//    //          NullspaceBasis(i) = pow(-1.0, i+7) * det;
//    //      }
//    //      //NullspaceBasis.Print();
//    //      NullspaceBasis.Normalize();
//    //      //cout<<NullspaceBasis.Norm()<<endl;
//    //      NullspaceProj = NullspaceBasis.MultTranspose(NullspaceBasis);
//    //      //NullspaceProj.Print();
//    //      NullspaceTorquesAP = JointTarget - Encoders.GetJointAngles();

//    //      cout << "NullspaceTorquesAP: "; //I need to recompile this too ... why is it not displaying ??
//    //      NullspaceTorquesAP.Print();

//    //      for(int i=0;i<NullspaceTorques.Size();i++){
//    //          NullspaceTorques(i) = NullspaceStiffness(i) * NullspaceTorquesAP(i);
//    //          NullspaceTorques(i) -= 0.5* NullspaceStiffness(i) * Encoders.GetJointVelocities()(i); // auto damping ???
//    //      }
//    //      //NullspaceTorques.Print();
//    //      NullspaceProj.Mult(NullspaceTorques,NullspaceTorquesAP);
//    //      //NullspaceTorquesAP.Print();
//    //      ControlTorque += NullspaceTorquesAP;
//    //  }


//}


void ImpedanceController::Update_e(const KDL::JntArray _jnts ){


  /// a) get Jacobian
  KDL::Jacobian kld_jac;
  KDL::Frame kdlf_frame;

  kld_jac.resize(kdl_chain.getNrOfJoints());

  /// Prepare solvers
  KDL::ChainJntToJacSolver kdlcjsolver_endeff = KDL::ChainJntToJacSolver(kdl_chain);
  KDL::ChainFkSolverPos_recursive kdlfksolver_endeff = KDL::ChainFkSolverPos_recursive(kdl_chain);

  /// Compute the Jacobian and the Cartesian Position
  kdlcjsolver_endeff.JntToJac(_jnts,kld_jac);
  kdlfksolver_endeff.JntToCart(_jnts,kdlf_frame);

  e_jacobian=kld_jac.data;
  e_jacobianT=e_jacobian.transpose();

  Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> > e_mapM(kdlf_frame.M.data);
  e_curr_orient = e_mapM;

  Eigen::Map< Eigen::Vector3d > e_mapV(kdlf_frame.p.data);
  e_curr_pos=e_mapV;

  e_temp3x3 = e_curr_orient.transpose();
  e_err_orient=e_des_orient*e_temp3x3;

  Eigen::AngleAxisd e_angleaxis(e_err_orient);
  e_err_axis=e_angleaxis.axis()*e_angleaxis.angle();

  // update the err vector containig the positional error and the orientation axis error concatenated. This format corresponds to the Jacobian. Alternative that needs some work: use quaternion representation for the orientation part. Then the jacobian needs to be rebuilt.
  for(int i=0;i<3;i++){
    e_err(i)=e_des_pos(i)-e_curr_pos(i);
    e_err(i+3)=e_err_axis(i);
  }

  e_jnt_pos_dot=(_jnts.data-e_jnt_pos_prev)/0.001; //TODO: replace the value by variable

  e_pos6_dot=e_jacobian*e_jnt_pos_dot; // careful: only damping on velocity ... // x_dot = J*q_dot // is it right ??
  e_err_dot=-e_pos6_dot;

  // kp and kd
  e_control_wrench=e_Kp*e_err;
  e_temp6x1=e_Kd*e_err_dot;
  e_control_wrench+=e_temp6x1;


  e_control_torque=e_jacobianT*e_control_wrench;


  //add Joint space damping
  e_joint_damping_torques=e_jointSpaceDamping.cwiseProduct(e_jnt_pos_dot);
  for (int i=0;i<e_joint_damping_torques.rows();i++){
    e_joint_damping_torques(i)=TRUNC(e_joint_damping_torques(i),-50.0,50.0);
  }

  if(bJointSpaceDamping)
    e_control_torque-=e_joint_damping_torques;

  e_jnt_pos_prev=_jnts.data;
  e_prev_pos=e_curr_pos;
  e_prev_des_pos=e_des_pos;

}

//void ImpedanceController::SetRobot(Robot* pRob,string targetLink){
//    pRobot=pRob;
//    bHaveRobot=true;
//    if(pRobot==NULL)
//        bHaveRobot=false;
//    if(bHaveRobot){
//        KinChain.SetRobot(pRobot);
//        //this was a nasty bug. The force was applied at the end of the kinematic chain, but it should be applied directly after the last joint
//        mLinkIndex = pRobot->GetLinkIndex(targetLink);
//        //KinChain.Create(0,0,pRobot->GetLinkIndex("TOOLPLATE"));
//        KinChain.Create(0,0,mLinkIndex);
//        //cout<<pRobot->GetLinkIndex("TOOLPLATE")<<pRobot->GetLinkIndex("TOOL")<<endl;
//        //mLinkIndex = pRobot->GetLinkIndex("TOOLPLATE");
//        //    exit(1);
//        //    LinksList hej = pRobot->GetLinks();
//        //    XmlTree tree;
//        //    tree.LoadFromFile("./packages/addons/WAMRobotModel/data/Robots/WAM/HapticBall/structure.xml");
//        //    tree.Print();
//        //    for(int i=0;i<pRobot->GetLinksCount();i++){
//        //        cout<<hej[i]->GetName()<<endl;
//        //    }
//        Encoders.SetSensorsList(pRobot->GetSensors());
//        Resize();
//    }
//    //KinChain.Update();
//    //KinChain.GetJacobian().Print();
//    //exit(1);
//}


void ImpedanceController::InitChain(KDL::Chain _chain){
  kdl_chain=_chain; // does it copy the whole chain or is it a link ? not sure ...


  Resize_e();

  // define which joints also ?
  // maybe not

}







//void ImpedanceController::Init(pXmlTree tree)
//{
//    if(tree==NULL)
//        return;


//    if(tree->GetName() != "ImpedanceController"){
//        return;
//    }
//    double * trans_stiff;
//    tree->GetArray("TranslationalStiffness",&trans_stiff);
//    for(int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            Kp(i,j) = trans_stiff[3*i+j];
//        }
//    }
//    double * trans_damp;
//    tree->GetArray("TranslationalDamping",&trans_damp);
//    for(int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            Kd(i,j) = trans_damp[3*i+j];
//        }
//    }
//    double * rot_stiff;
//    tree->GetArray("RotationalStiffness",&rot_stiff);
//    for(int i=3;i<6;i++){
//        for(int j=3;j<6;j++){
//            Kp(i,j) = rot_stiff[3*(i-3)+(j-3)];
//        }
//    }
//    double * rot_damp;
//    tree->GetArray("RotationalDamping",&rot_damp);
//    for(int i=3;i<6;i++){
//        for(int j=3;j<6;j++){
//            Kd(i,j) = rot_damp[3*(i-3)+(j-3)];
//        }
//    }

//    double * js_damp;
//    tree->GetArray("JointSpaceDamping",&js_damp);
//    for(int i=0;i<jointSpaceDamping.Size();i++){
//        jointSpaceDamping(i)  = js_damp[i];
//    }
////    Kp.Print();
////    Kd.Print();
////    jointSpaceDamping.Print();



////    exit(1);




//}


void ImpedanceController::GetOutput(Vector & result){

  if(result.Size()==ControlTorque.Size())
    result = ControlTorque;
  else{
    //        cout<<"dimension mismatch when getting output, resizing vector!"<<endl;
    result.Resize(ControlTorque.Size());
    result = ControlTorque;
  }

}

void ImpedanceController::GetOutput_e(Eigen::Matrix<double,Eigen::Dynamic,1> &result){

  if(result.rows()==e_control_torque.rows())
    result = e_control_torque;
  else{
    //        cout<<"dimension mismatch when getting output, resizing vector!"<<endl;
    result.resize(e_control_torque.rows()); // not sure about this one
    result = e_control_torque;
  }

}


void ImpedanceController::GetError_e(Eigen::Matrix<double,Eigen::Dynamic,1> &error){

  error=e_err;

}


//void ImpedanceController::AddOutput(Vector & result){

//  if(result.Size()==ControlTorque.Size())
//    result += ControlTorque;
//  else{
//    cout<<"dimension mismatch when getting output, resizing vector!"<<endl;
//    result.Resize(ControlTorque.Size());
//    result += ControlTorque;
//  }


//}

