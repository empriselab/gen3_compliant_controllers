// pinocchio headers
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <gen3_compliant_controllers/TaskSpaceCompliantController.hpp>
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

TaskSpaceCompliantController::TaskSpaceCompliantController() : MultiInterfaceController(true) // allow_optional_interfaces
{
}

//=============================================================================
TaskSpaceCompliantController::~TaskSpaceCompliantController()
{
}

bool TaskSpaceCompliantController::init(hardware_interface::RobotHW* robot, ros::NodeHandle& n)
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

  const auto numControlledDofs = mModel->nv;
  mControlledJointHandles.resize(numControlledDofs);
  for (size_t idof = 0; idof < numControlledDofs; ++idof)
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

  mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
  mExtendedJointsGravity = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);

  mCount = 0;
  mNumControlledDofs = mModel->nv;

  mJointStiffnessMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mJointStiffnessMatrix.setZero();

  mRotorInertiaMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mRotorInertiaMatrix.setZero();

  mFrictionL.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionL.setZero();

  mFrictionLp.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionLp.setZero();

  mTaskKMatrix.resize(6, 6);
  mTaskKMatrix.setZero();

  mTaskDMatrix.resize(6, 6);
  mTaskDMatrix.setZero();

  if (mNumControlledDofs == 6)
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 4, 4, 4;
  }
  else
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 5, 4, 4, 4;
  }
  mTaskKMatrix.diagonal() << 200, 200, 200, 100, 100, 100;
  mTaskDMatrix.diagonal() << 40, 40, 40, 20, 20, 20;

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mZeros.resize(numControlledDofs);
  mZeros.setZero();

  mName = internal::getLeafNamespace(n);

  // Initialize controlled joints
  std::string param_name = "joints";
  if (!n.getParam(param_name, mJointNames))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }

  // ROS API: Subscribed topics
  mSubCommand = n.subscribe<moveit_msgs::CartesianTrajectoryPoint>("command", 1, &TaskSpaceCompliantController::commandCallback, this);

  // Dynamic reconfigure server
  f = boost::bind(&TaskSpaceCompliantController::dynamicReconfigureCallback, this, _1, _2);
  server.setCallback(f);

  ROS_INFO("TaskSpaceCompliantController initialized successfully");
  return true;
}

//=============================================================================
void TaskSpaceCompliantController::dynamicReconfigureCallback(gen3_compliant_controllers::TaskSpaceCompliantControllerConfig& config, uint32_t level)
{
  if (mNumControlledDofs == 6)
  {
    mJointStiffnessMatrix.diagonal() << config.j_0, config.j_1, config.j_2, config.j_3, config.j_4, config.j_5;
    mRotorInertiaMatrix.diagonal() << config.b_0, config.b_1, config.b_2, config.b_3, config.b_4, config.b_5;
    mFrictionL.diagonal() << config.l_0, config.l_1, config.l_2, config.l_3, config.l_4, config.l_5;
    mFrictionLp.diagonal() << config.lp_0, config.lp_1, config.lp_2, config.lp_3, config.lp_4, config.lp_5;
  }
  else
  {
    mJointStiffnessMatrix.diagonal() << config.j_0, config.j_1, config.j_2, config.j_3, config.j_4, config.j_5, config.j_6;
    mRotorInertiaMatrix.diagonal() << config.b_0, config.b_1, config.b_2, config.b_3, config.b_4, config.b_5, config.b_6;
    mFrictionL.diagonal() << config.l_0, config.l_1, config.l_2, config.l_3, config.l_4, config.l_5, config.l_6;
    mFrictionLp.diagonal() << config.lp_0, config.lp_1, config.lp_2, config.lp_3, config.lp_4, config.lp_5, config.lp_6;
  }
  mTaskKMatrix.diagonal() << config.k_x, config.k_y, config.k_z, config.k_roll, config.k_pitch, config.k_yaw;
  mTaskDMatrix.diagonal() << config.d_x, config.d_y, config.d_z, config.d_roll, config.d_pitch, config.d_yaw;
}

//=============================================================================
void TaskSpaceCompliantController::starting(const ros::Time& time)
{

  mExecuteDefaultCommand = true;

  ROS_DEBUG_STREAM("Initialized desired position: " << mDesiredPosition.transpose());
  ROS_DEBUG_STREAM("Initialized desired velocity: " << mDesiredVelocity.transpose());

  ROS_DEBUG("Reset PID.");
}

//=============================================================================
void TaskSpaceCompliantController::stopping(const ros::Time& time)
{
  ROS_DEBUG("Stopping controller.");
}

//=============================================================================
void TaskSpaceCompliantController::update(const ros::Time& time, const ros::Duration& period)
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
    Eigen::VectorXd q_pin_desired = joint_ros_to_pinocchio(mDesiredPosition, *mModel);
    pinocchio::forwardKinematics(*mModel, *mData, q_pin_desired, mZeros);
    pinocchio::updateFramePlacement(*mModel, *mData, mEENode);
    mDesiredEETransform = mData->oMf[mEENode].toHomogeneousMatrix_impl();
    auto tmp = mDesiredEETransform.linear();
    // make first and second column negative to account for axis convention
    tmp.col(0) = -tmp.col(0);
    tmp.col(1) = -tmp.col(1);
    mExecuteDefaultCommand = false;
  }
  else
  {
    moveit_msgs::CartesianTrajectoryPoint& command = *mCommandsBuffer.readFromRT();
    if (command.point.pose.position.x == 0.0 && command.point.pose.position.y == 0.0 && command.point.pose.position.z == 0.0)
    {
      ROS_WARN_STREAM_NAMED(mName, "Received command with zero position, skipping.");
    }
    else
    {
      geometry_msgs::Pose command_pose = command.point.pose;
      tf::poseMsgToEigen(command_pose, mDesiredEETransform);
      auto tmp = mDesiredEETransform.linear();
      tmp.col(0) = -tmp.col(0);
      tmp.col(1) = -tmp.col(1);
    }
  }

  if (!mExtendedJoints->mIsInitialized)
  {
    mLastDesiredPosition = mDesiredPosition;
    mLastDesiredEETransform = mDesiredEETransform;
    mExtendedJoints->initializeExtendedJointPosition(mDesiredPosition);
    mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
    mNominalThetaPrev = mExtendedJoints->getExtendedJoint();
    mNominalThetaDotPrev = mCurrentVelocity;
    mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
    mTrueDesiredVelocity = mDesiredVelocity;
    mTrueDesiredEETransform = mDesiredEETransform;
  }

  if (mDesiredPosition != mLastDesiredPosition || !mDesiredEETransform.isApprox(mLastDesiredEETransform, 0.0001) && mCurrentPosition != mDesiredPosition)
  {
    mLastDesiredPosition = mDesiredPosition;
    mLastDesiredEETransform = mDesiredEETransform;
    mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
    mTrueDesiredVelocity = mDesiredVelocity;
    mTrueDesiredEETransform = mDesiredEETransform;
  }

  {
    mExtendedJointsGravity->mIsInitialized = false;
    mExtendedJointsGravity->initializeExtendedJointPosition(mDesiredPosition);
    mExtendedJointsGravity->estimateExtendedJoint(mExtendedJointsGravity->mLastDesiredPosition);
    mCurrentTheta = mExtendedJointsGravity->getExtendedJoint();

    mGravity = pinocchio::computeGeneralizedGravity(*mModel, *mData, joint_ros_to_pinocchio(mCurrentTheta, *mModel));

    // compute quasi-static estimate of the link side position
    // input value is motor side angle theta not link side angle(q);
    // Number of iteration can be modified by editing i (recommend is 1 or 2
    // for real-time computing)
    // NOTE: currently not being used

    // int iteration = 1; // number of iteration
    // Eigen::VectorXd qs_estimate_link_pos(numControlledDofs);
    // qs_estimate_link_pos = mNominalThetaPrev;

    // for (int i=0; i<iteration; i++)
    // {
    // 	Eigen::VectorXd q_pin = joint_ros_to_pinocchio(qs_estimate_link_pos,
    // *mModel); 	mGravity = pinocchio::computeGeneralizedGravity(*mModel,
    // *mData, q_pin);
    //     qs_estimate_link_pos = mNominalThetaPrev -
    //     mJointStiffnessMatrix.inverse()*mGravity;
    // }
    // Eigen::VectorXd q_pin = joint_ros_to_pinocchio(qs_estimate_link_pos,
    // *mModel); mQuasiGravity = pinocchio::computeGeneralizedGravity(*mModel,
    // *mData, q_pin);
  }

  mExtendedJoints->estimateExtendedJoint(mCurrentPosition);
  mCurrentTheta = mExtendedJoints->getExtendedJoint();

  mDesiredTheta = mTrueDesiredPosition + mJointStiffnessMatrix.inverse() * mGravity;
  mDesiredThetaDot = mTrueDesiredVelocity;

  // Compute error
  Eigen::VectorXd dart_error(6);
  Eigen::MatrixXd dart_nominal_jacobian(6, mNumControlledDofs);
  {
    Eigen::Quaterniond ee_quat_d(mTrueDesiredEETransform.linear());

    Eigen::VectorXd q_pin_nominal_prev = joint_ros_to_pinocchio(mNominalThetaPrev, *mModel);
    pinocchio::computeJointJacobians(*mModel, *mData, q_pin_nominal_prev);
    pinocchio::updateFramePlacement(*mModel, *mData, mEENode);
    mNominalEETransform = mData->oMf[mEENode].toHomogeneousMatrix_impl();
    auto tmp2 = mNominalEETransform.linear();
    // make first and second column negative to account for axis convention
    tmp2.col(0) = -tmp2.col(0);
    tmp2.col(1) = -tmp2.col(1);

    Eigen::Quaterniond nominal_ee_quat(mNominalEETransform.linear());

    pinocchio::getFrameJacobian(*mModel, *mData, mEENode, pinocchio::LOCAL_WORLD_ALIGNED, dart_nominal_jacobian);
    dart_error.head(3) << mNominalEETransform.translation() - mTrueDesiredEETransform.translation(); // positional error

    if (ee_quat_d.coeffs().dot(nominal_ee_quat.coeffs()) < 0.0)
    {
      nominal_ee_quat.coeffs() << -nominal_ee_quat.coeffs();
    }

    Eigen::Quaterniond error_qtn(nominal_ee_quat.inverse() * ee_quat_d);
    dart_error.tail(3) << error_qtn.x(), error_qtn.y(), error_qtn.z();
    dart_error.tail(3) << -mNominalEETransform.linear() * dart_error.tail(3);
  }

  mTaskEffort = dart_nominal_jacobian.transpose() * (-mTaskKMatrix * dart_error - mTaskDMatrix * (dart_nominal_jacobian * mNominalThetaDotPrev));

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
    if (mCount % 10 == 0)
      std::cout << "Initializing controller: " << mCount << std::endl;
  }

  for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof)
  {
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(mCommandEffort[idof]);
  }
}

void TaskSpaceCompliantController::commandCallback(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
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
  mExecuteDefaultCommand = false;
}

//=============================================================================
bool TaskSpaceCompliantController::shouldAcceptRequests()
{
  return isRunning();
}

//=============================================================================
// Default for virtual function is do nothing. DO NOT EDIT
bool TaskSpaceCompliantController::shouldStopExecution(std::string& message)
{
  return false;
}

} // namespace gen3_compliant_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(gen3_compliant_controllers::TaskSpaceCompliantController, controller_interface::ControllerBase)
