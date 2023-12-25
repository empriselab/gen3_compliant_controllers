// pinocchio headers
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <gen3_compliant_controllers/JointSpaceCompliantController.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace gen3_compliant_controllers {

namespace internal {

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace internal

JointSpaceCompliantController::JointSpaceCompliantController() : MultiInterfaceController(true) // allow_optional_interfaces
{
}

//=============================================================================
JointSpaceCompliantController::~JointSpaceCompliantController()
{
}

bool JointSpaceCompliantController::init(hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{

  using hardware_interface::EffortJointInterface;
  using hardware_interface::JointStateInterface;

  mNodeHandle.reset(new ros::NodeHandle{n});

  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto& param : jointParameters)
  {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "effort")
    {
      ROS_ERROR_STREAM(
          "Joint '" << param.mName
                    << "' is not effort-controlled and cannot be "
                       "used in a gravity compensation controller");
      return false;
    }
  }

  std::string parameterName;
  mNodeHandle->param<std::string>("robot_description_parameter", parameterName, "/robot_description");

  // Load the URDF from the parameter server.
  std::string robotDescription;
  if (!mNodeHandle->getParam(parameterName, robotDescription))
  {
    ROS_ERROR_STREAM("Failed loading URDF from '" << parameterName << "' parameter.");
    return false;
  }
  mModel = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModelFromXML(robotDescription, *mModel.get());
  if (!mModel)
    return false;

  mData.reset(new pinocchio::Data{*mModel.get()});

  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface)
  {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mJointStateUpdater.reset(new JointStateUpdater{mModel, jointStateInterface});

  const auto effortJointInterface = robot->get<EffortJointInterface>();
  if (!effortJointInterface)
  {
    ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
    return false;
  }

  mNumControlledDofs = mModel->nv;
  mControlledJointHandles.resize(mNumControlledDofs);
  for (size_t idof = 0; idof < mNumControlledDofs; ++idof)
  {
    const auto dofName = mModel->names[idof + 1];
    try
    {
      auto handle = effortJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Unable to get interface of type 'EffortJointInterface' for joint '" << dofName << "'.");
      return false;
    }
  }
  std::cout << "ControlledJointHandles created" << std::endl;

  std::string mEENodeName;
  n.getParam("/end_effector", mEENodeName);
  mEENode = mModel->getFrameId(mEENodeName);

  mExtendedJoints = new ExtendedJointPosition(mNumControlledDofs, 3 * M_PI / 2);

  mCount = 0;

  mJointStiffnessMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mJointStiffnessMatrix.setZero();

  mRotorInertiaMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mRotorInertiaMatrix.setZero();

  mFrictionL.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionL.setZero();

  mFrictionLp.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionLp.setZero();

  mJointKMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mJointKMatrix.setZero();

  mJointDMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mJointDMatrix.setZero();

  if (mNumControlledDofs == 6)
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 4, 4, 4;
    mJointKMatrix.diagonal() << 10, 10, 10, 10, 10, 10;
    mJointDMatrix.diagonal() << 2, 2, 2, 2, 2, 2;
  }
  else
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 5, 4, 4, 4;
    mJointKMatrix.diagonal() << 10, 10, 10, 10, 10, 10, 10;
    mJointDMatrix.diagonal() << 2, 2, 2, 2, 2, 2, 2;
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(mNumControlledDofs);
  mDesiredVelocity.resize(mNumControlledDofs);
  mZeros.resize(mNumControlledDofs);
  mSubCommand = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, &JointSpaceCompliantController::commandCallback, this);

  // Dynamic reconfigure server
  f = boost::bind(&JointSpaceCompliantController::dynamicReconfigureCallback, this, _1, _2);
  server.setCallback(f);

  ROS_INFO("JointSpaceCompliantController initialized successfully");
  return true;
}

//=============================================================================
void JointSpaceCompliantController::dynamicReconfigureCallback(gen3_compliant_controllers::JointSpaceCompliantControllerConfig& config, uint32_t level)
{
  if (mNumControlledDofs == 6)
  {
    mJointStiffnessMatrix.diagonal() << config.j_0, config.j_1, config.j_2, config.j_3, config.j_4, config.j_5;
    mRotorInertiaMatrix.diagonal() << config.b_0, config.b_1, config.b_2, config.b_3, config.b_4, config.b_5;
    mFrictionL.diagonal() << config.l_0, config.l_1, config.l_2, config.l_3, config.l_4, config.l_5;
    mFrictionLp.diagonal() << config.lp_0, config.lp_1, config.lp_2, config.lp_3, config.lp_4, config.lp_5;
    mJointKMatrix.diagonal() << config.k_0, config.k_1, config.k_2, config.k_3, config.k_4, config.k_5;
    mJointDMatrix.diagonal() << config.d_0, config.d_1, config.d_2, config.d_3, config.d_4, config.d_5;
  }
  else
  {
    mJointStiffnessMatrix.diagonal() << config.j_0, config.j_1, config.j_2, config.j_3, config.j_4, config.j_5, config.j_6;
    mRotorInertiaMatrix.diagonal() << config.b_0, config.b_1, config.b_2, config.b_3, config.b_4, config.b_5, config.b_6;
    mFrictionL.diagonal() << config.l_0, config.l_1, config.l_2, config.l_3, config.l_4, config.l_5, config.l_6;
    mFrictionLp.diagonal() << config.lp_0, config.lp_1, config.lp_2, config.lp_3, config.lp_4, config.lp_5, config.lp_6;
    mJointKMatrix.diagonal() << config.k_0, config.k_1, config.k_2, config.k_3, config.k_4, config.k_5, config.k_6;
    mJointDMatrix.diagonal() << config.d_0, config.d_1, config.d_2, config.d_3, config.d_4, config.d_5, config.d_6;
  }
}

//=============================================================================
void JointSpaceCompliantController::starting(const ros::Time& time)
{

  mExecuteDefaultCommand = true;

  ROS_DEBUG_STREAM("Initialized desired position: " << mDesiredPosition.transpose());
  ROS_DEBUG_STREAM("Initialized desired velocity: " << mDesiredVelocity.transpose());

  ROS_DEBUG("Reset PID.");
}

//=============================================================================
void JointSpaceCompliantController::stopping(const ros::Time& time)
{
}

//=============================================================================
void JointSpaceCompliantController::update(const ros::Time& time, const ros::Duration& period)
{

  auto current_time = std::chrono::high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(current_time - mLastTimePoint);

  // std::cout << "Controller Frequency: " << 1000000.0 / duration.count()
  //           << std::endl;
  mLastTimePoint = std::chrono::high_resolution_clock::now();

  mJointStateUpdater->update();
  mCurrentPosition = mJointStateUpdater->mCurrentPosition;
  mCurrentVelocity = mJointStateUpdater->mCurrentVelocity;
  mCurrentEffort = mJointStateUpdater->mCurrentEffort;

  if (mExecuteDefaultCommand.load())
  {
    mDesiredPosition = mCurrentPosition;
    mDesiredVelocity.setZero();
    mExecuteDefaultCommand = false;
  }
  else
  {
    trajectory_msgs::JointTrajectoryPoint command = *mCommandsBuffer.readFromRT();
    if (command.positions.size() != mNumControlledDofs || command.velocities.size() != mNumControlledDofs)
    {
      // assume last command
      ROS_WARN_STREAM("Received command with wrong size. Assuming last command.");
    }
    else
    {
      mDesiredPosition = Eigen::VectorXd::Map(&command.positions[0], command.positions.size());
      mDesiredVelocity = Eigen::VectorXd::Map(&command.velocities[0], command.velocities.size());
    }
  }

  if (!mExtendedJoints->mIsInitialized)
  {
    mLastDesiredPosition = mDesiredPosition;
    mExtendedJoints->initializeExtendedJointPosition(mCurrentPosition);
    mExtendedJoints->estimateExtendedJoint(mCurrentPosition);
    mNominalThetaPrev = mExtendedJoints->getExtendedJoint();
    mNominalThetaDotPrev = mCurrentVelocity;
    mDesiredPosition = mExtendedJoints->getExtendedJoint();
  }

  if (mDesiredPosition != mLastDesiredPosition && mCurrentPosition != mDesiredPosition)
  {
    mLastDesiredPosition = mDesiredPosition;
    mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
    mDesiredPosition = mExtendedJoints->getExtendedJoint();
  }

  mExtendedJoints->estimateExtendedJoint(mCurrentPosition);
  mCurrentTheta = mExtendedJoints->getExtendedJoint();

  mGravity = pinocchio::computeGeneralizedGravity(*mModel, *mData, joint_ros_to_pinocchio(mCurrentTheta, *mModel));

  mDesiredTheta = mDesiredPosition + mJointStiffnessMatrix.inverse() * mGravity;
  mDesiredThetaDot = mDesiredVelocity;

  mTaskEffort = -mJointKMatrix * (mNominalThetaPrev - mDesiredTheta) - mJointDMatrix * (mNominalThetaDotPrev - mDesiredThetaDot);

  double step_time;
  step_time = 0.001;

  mNominalThetaDDot = mRotorInertiaMatrix.inverse() * (mTaskEffort + mGravity + mCurrentEffort); // mCurrentEffort is negative of what is required here
  mNominalThetaDot = mNominalThetaDotPrev + mNominalThetaDDot * step_time;
  mNominalTheta = mNominalThetaPrev + mNominalThetaDot * step_time;

  mNominalThetaPrev = mNominalTheta;
  mNominalThetaDotPrev = mNominalThetaDot;

  mNominalFriction = mRotorInertiaMatrix * mFrictionL * ((mNominalThetaDotPrev - mCurrentVelocity) + mFrictionLp * (mNominalThetaPrev - mCurrentTheta));

  mCommandEffort = mTaskEffort + mNominalFriction;

  if (mCount < 50)
  {
    mCommandEffort = Eigen::VectorXd::Zero(mNumControlledDofs);
    mCount++;
    if (mCount % 200 == 0)
      std::cout << "Initializing controller: " << mCount << std::endl;
    return;
  }

  for (size_t idof = 0; idof < mNumControlledDofs; ++idof)
  {
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(mCommandEffort[idof]);
  }
}

void JointSpaceCompliantController::commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
{
  // Preconditions
  if (!shouldAcceptRequests())
  {
    ROS_ERROR_STREAM_NAMED(mName, "Can't accept new commands. Controller is not running.");
    return;
  }

  if (!msg)
  {
    ROS_WARN_STREAM_NAMED(mName, "Received null-pointer message, skipping.");
    return;
  }

  mCommandsBuffer.writeFromNonRT(*msg);
  std::cout << "Received command" << std::endl;
  mExecuteDefaultCommand = false;
}

//=============================================================================
bool JointSpaceCompliantController::shouldAcceptRequests()
{
  return isRunning();
}

//=============================================================================
// Default for virtual function is do nothing. DO NOT EDIT
bool JointSpaceCompliantController::shouldStopExecution(std::string& message)
{
  return false;
}

} // namespace gen3_compliant_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(gen3_compliant_controllers::JointSpaceCompliantController, controller_interface::ControllerBase)