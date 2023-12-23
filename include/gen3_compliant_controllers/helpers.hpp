#ifndef GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_
#define GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_

#include <pinocchio/algorithm/model.hpp>

#include <string>
#include <typeinfo>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

namespace gen3_compliant_controllers {

//=============================================================================
struct JointParameter
{
  std::string mName;
  std::string mType;
};

//=============================================================================
class JointStateUpdater final
{
public:
  JointStateUpdater(
      std::shared_ptr<pinocchio::Model> model,
      hardware_interface::JointStateInterface* jointStateInterface);

  void update();

  Eigen::VectorXd mActualPosition;
  Eigen::VectorXd mActualVelocity;
  Eigen::VectorXd mActualEffort;
  Eigen::VectorXd mPinActualPosition;

private:
  std::shared_ptr<pinocchio::Model> mModel;
  Eigen::VectorXd mDefaultPosition;
  Eigen::VectorXd mDefaultVelocity;
  Eigen::VectorXd mDefaultEffort;
  Eigen::VectorXd mPinDefaultPosition;
  std::vector<hardware_interface::JointStateHandle> mHandles;
};

//=============================================================================
class ExtendedJointPosition
{
private:
  unsigned int mNumberOfInput;
  double mThresholdOfChange;

  Eigen::VectorXd mInitQ;
  Eigen::VectorXd mExtendedQ;
  Eigen::VectorXd mPreviousSensorQ;

public:
  ExtendedJointPosition(
      unsigned int number_of_input_args, double threshold_of_change_args);

  void initializeExtendedJointPosition(const Eigen::VectorXd& init_q_args);

  double normalizeJointPosition(double input);

  Eigen::VectorXd normalizeJointPosition(const Eigen::VectorXd& input);

  void estimateExtendedJoint(const Eigen::VectorXd& current_sensor_q);

  Eigen::VectorXd getExtendedJoint();

  bool mIsInitialized = false;
  Eigen::VectorXd mLastDesiredPosition;

  Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat, double eps = 1e-6);

  Eigen::MatrixXd computeEEMassMatrix(
      Eigen::MatrixXd mMassMatrix, Eigen::VectorXd q, Eigen::MatrixXd J);
};

//=============================================================================
Eigen::VectorXd joint_ros_to_pinocchio(
    Eigen::VectorXd& q, pinocchio::Model& model);

//=============================================================================
std::vector<JointParameter> loadJointsFromParameter(
    const ros::NodeHandle& nodeHandle,
    const std::string& jointsParameter,
    const std::string& defaultType);

} // namespace gen3_compliant_controllers

#endif // ifndef GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_
