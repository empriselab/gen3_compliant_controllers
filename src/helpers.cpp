#include <gen3_compliant_controllers/helpers.hpp>

namespace gen3_compliant_controllers {

//=============================================================================
JointStateUpdater::JointStateUpdater(
    std::shared_ptr<pinocchio::Model> model,
    hardware_interface::JointStateInterface* jointStateInterface)
  : mModel(model)
  , mActualPosition(model->nv)
  , mActualVelocity(model->nv)
  , mActualEffort(model->nv)
  , mPinActualPosition(model->nq)
  , mDefaultPosition(model->nv)
  , mDefaultVelocity(model->nv)
  , mDefaultEffort(model->nv)
  , mPinDefaultPosition(model->nq)
{
  std::set<std::string> missingJointNames;

  mHandles.reserve(model->nv);

  for (size_t idof = 0; idof < model->names.size() - 1; ++idof)
  {
    const auto dofName = model->names[idof + 1];

    mDefaultPosition[idof] = 0.;
    mDefaultVelocity[idof] = 0.;
    mDefaultEffort[idof] = 0.;

    hardware_interface::JointStateHandle handle;
    try
    {
      handle = jointStateInterface->getHandle(dofName);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      missingJointNames.emplace(dofName);

      // Use the default position, velocity, and effort.
      handle = hardware_interface::JointStateHandle{
          dofName,
          &mDefaultPosition[idof],
          &mDefaultVelocity[idof],
          &mDefaultEffort[idof]};
    }

    mHandles.emplace_back(handle);
  }

  if (mHandles.empty())
  {
    std::stringstream msg;
    msg << "Failed to get JointStateHandles for " << missingJointNames.size()
        << " joints. The following joints will be assumed to have their"
        << " position and velocity for dynamics calculations:";

    for (const auto& dofName : missingJointNames)
      msg << " '" << dofName << "'";

    ROS_WARN_STREAM(msg.str());
  }

  mPinDefaultPosition = joint_ros_to_pinocchio(mDefaultPosition, *model.get());
}

//=============================================================================
void JointStateUpdater::update()
{
  for (size_t idof = 0; idof < mModel->nv; ++idof)
  {
    const auto& jointStateHandle = mHandles[idof];
    mActualPosition(idof) = jointStateHandle.getPosition();
    if (mActualPosition[idof] < 0)
      mActualPosition[idof] += 2 * M_PI;
    mActualVelocity[idof] = jointStateHandle.getVelocity();
    mActualEffort[idof] = jointStateHandle.getEffort();
  }
}

//=============================================================================
ExtendedJointPosition::ExtendedJointPosition(
    unsigned int numberOfInput_args, double threshold_of_change_args)
{
  mNumberOfInput = numberOfInput_args;
  mThresholdOfChange = threshold_of_change_args;
  mInitQ.resize(mNumberOfInput, 1);
  mExtendedQ.resize(mNumberOfInput, 1);
  mPreviousSensorQ.resize(mNumberOfInput, 1);
}

//=============================================================================
void ExtendedJointPosition::initializeExtendedJointPosition(
    const Eigen::VectorXd& init_q_args)
{
  if (mIsInitialized == false)
  {
    mLastDesiredPosition = init_q_args;
    mInitQ = normalizeJointPosition(init_q_args);
    mExtendedQ = mInitQ;
    mPreviousSensorQ = mInitQ;
    mIsInitialized = true;
  }
}

//=============================================================================
double ExtendedJointPosition::normalizeJointPosition(double input)
{
  /*
   * Move the joint position inside the [-pi,pi).
   * args : double -> 1d case
   * args : VectorXd -> multiple dimension(=dof) case
   */
  // double output = input;
  while (input > M_PI)
  {
    input -= 2 * M_PI;
  }
  while (input <= -M_PI)
  {
    input += 2 * M_PI;
  }
  return input;
}

//=============================================================================
Eigen::VectorXd ExtendedJointPosition::normalizeJointPosition(
    const Eigen::VectorXd& input)
{
  Eigen::VectorXd output = input;
  for (int i = 0; i < mNumberOfInput; ++i)
  {
    output[i] = normalizeJointPosition(input[i]);
  }
  return output;
}

//=============================================================================
void ExtendedJointPosition::estimateExtendedJoint(
    const Eigen::VectorXd& current_sensor_q)
{
  for (int i = 0; i < mNumberOfInput; ++i)
  {
    if (abs(current_sensor_q[i] - mPreviousSensorQ(i)) >= mThresholdOfChange)
    {
      mExtendedQ(i) += normalizeJointPosition(current_sensor_q[i])
                       - normalizeJointPosition(mPreviousSensorQ(i));
    }
    else
    {
      int howMuchRotate;
      if (mExtendedQ(i) >= 0)
      {
        howMuchRotate = static_cast<int>(mExtendedQ(i) / (2 * M_PI));
      }
      else
      { //=============================================================================
        howMuchRotate = static_cast<int>(mExtendedQ(i) / (2 * M_PI)) - 1;
      }
      mExtendedQ(i) = (double)howMuchRotate * (2 * M_PI) + current_sensor_q[i];
    }
  }
  mPreviousSensorQ = current_sensor_q;
}

//=============================================================================
Eigen::VectorXd ExtendedJointPosition::getExtendedJoint()
{
  return mExtendedQ;
}

//=============================================================================
Eigen::MatrixXd ExtendedJointPosition::pseudoinverse(
    const Eigen::MatrixXd& mat, double eps)
{
  if (mat.rows() == mat.cols() && mat.determinant() > eps)
    return mat.inverse();

  else
  {
    if (mat.cols() == 1)
    {
      if (mat.isApproxToConstant(0))
      {
        return Eigen::VectorXd::Zero(mat.rows());
      }

      return mat.transpose() / (pow(mat.norm(), 2));
    }

    /// Use SVD decomposition.
    Eigen::JacobiSVD<Eigen::MatrixXd> jacSVD(
        mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = jacSVD.matrixU();
    Eigen::MatrixXd V = jacSVD.matrixV();
    Eigen::VectorXd S = jacSVD.singularValues();

    Eigen::MatrixXd S_inv(Eigen::MatrixXd::Zero(mat.cols(), mat.rows()));

    for (int i = 0; i < S.rows(); i++)
    {
      if (S(i) > eps)
      {
        S_inv(i, i) = 1.0 / S(i);
      }
      else
      {
        S_inv(i, i) = 0;
      }
    }

    return V * S_inv * U.transpose();
  }
}

//=============================================================================
Eigen::MatrixXd ExtendedJointPosition::computeEEMassMatrix(
    Eigen::MatrixXd mMassMatrix, Eigen::VectorXd q, Eigen::MatrixXd J)
{
  Eigen::MatrixXd Mq_inv = pseudoinverse(mMassMatrix, 1e-6);
  Eigen::MatrixXd Mx_inv = J * (Mq_inv * J.transpose());
  Eigen::JacobiSVD<Eigen::MatrixXd> jacSVD(
      Mx_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // Mx_inv, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = jacSVD.matrixU();
  Eigen::MatrixXd V = jacSVD.matrixV();
  Eigen::VectorXd S = jacSVD.singularValues();

  Eigen::MatrixXd S_inv(Eigen::MatrixXd::Zero(Mx_inv.cols(), Mx_inv.rows()));
  for (int i = 0; i < S.rows(); i++)
  {
    if (S(i) > 0.0025)
    {
      S_inv(i, i) = 1.0 / S(i);
    }
    else
    {
      S_inv(i, i) = 0;
    }
  }

  Eigen::MatrixXd Mx = V * S_inv * U.transpose();
  return Mx;
}

//=============================================================================
Eigen::VectorXd joint_ros_to_pinocchio(
    Eigen::VectorXd& q, pinocchio::Model& model)
{
  Eigen::VectorXd q_pin(model.nq);
  // convert ROS joint config to pinocchio config
  for (int i = 0; i < model.joints.size() - 1; i++)
  {
    int jidx = model.getJointId(model.names[i + 1]);
    int qidx = model.idx_qs[jidx];
    // nqs[i] is 2 for continuous joints in pinocchio
    if (model.nqs[jidx] == 2)
    {
      q_pin[qidx] = std::cos(q[i]);
      q_pin[qidx + 1] = std::sin(q[i]);
    }
    else
    {
      q_pin[qidx] = q[i];
    }
  }
  return q_pin;
}

//=============================================================================
std::vector<JointParameter> loadJointsFromParameter(
    const ros::NodeHandle& nodeHandle,
    const std::string& jointsParameter,
    const std::string& defaultType)
{
  using XmlRpc::XmlRpcValue;

  static const std::vector<JointParameter> emptyResult;

  XmlRpcValue jointsXml;
  if (!nodeHandle.getParam("joints", jointsXml))
  {
    ROS_ERROR_STREAM(
        "Parameter '" << nodeHandle.getNamespace() << "/joints' is required.");
  }
  //=============================================================================
  if (jointsXml.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM(
        "Parameter '" << nodeHandle.getNamespace()
                      << "/joints' is not an array.");
    return emptyResult;
  }

  std::vector<JointParameter> output;
  for (int i = 0; i < jointsXml.size(); ++i)
  {
    JointParameter jointParameters;
    auto& jointXml = jointsXml[i];

    // Simple case where everything is effort-controlled.
    if (jointXml.getType() == XmlRpcValue::TypeString)
    {
      jointParameters.mName = static_cast<std::string>(jointXml);
      jointParameters.mType = defaultType;
    }
    // Advanced case where there are heterogeneous actuator types.
    else if (jointXml.getType() == XmlRpcValue::TypeStruct)
    {
      auto& nameXml = jointXml["name"];
      if (nameXml.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM(
            "Parameter '" << nodeHandle.getNamespace() << "/joints[" << i
                          << "]/name' is not a string.");
        return emptyResult;
      }
      jointParameters.mName = static_cast<std::string>(nameXml);

      auto& typeXml = jointXml["type"];
      if (typeXml.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM(
            "Parameter '" << nodeHandle.getNamespace() << "/joints[" << i
                          << "]/type' is not a string.");
        return emptyResult;
      }
      jointParameters.mType = static_cast<std::string>(typeXml);
    }
    else
    {
      ROS_ERROR_STREAM(
          "Parameter '" << nodeHandle.getNamespace() << "/joints[" << i
                        << "]' is not a struct.");
      return emptyResult;
    }

    output.emplace_back(jointParameters);
  }

  return output;
}

} // namespace gen3_compliant_controllers
