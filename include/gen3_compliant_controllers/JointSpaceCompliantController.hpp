#ifndef GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H
#define GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H

// pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/sample-models.hpp>

#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"

// ROS messages
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <chrono>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
using namespace std::chrono;

#include <dynamic_reconfigure/server.h>
#include <gen3_compliant_controllers/JointSpaceCompliantControllerConfig.h>
#include <gen3_compliant_controllers/helpers.hpp>

namespace gen3_compliant_controllers {
class JointSpaceCompliantController
  : public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface,
        hardware_interface::JointStateInterface>
{
public:
  JointSpaceCompliantController();
  ~JointSpaceCompliantController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  std::unique_ptr<JointStateUpdater> mJointStateUpdater;
  bool shouldAcceptRequests();
  bool shouldStopExecution(std::string& message);

  realtime_tools::RealtimeBuffer<trajectory_msgs::JointTrajectoryPoint>
      mCommandsBuffer;
  std::atomic_bool mExecuteDefaultCommand;

  std::string mName;                    ///< Controller name.
  std::vector<std::string> mJointNames; ///< Controlled joint names

  // pinocchio objects
  std::shared_ptr<pinocchio::Model> mModel;
  std::unique_ptr<pinocchio::Data> mData;
  pinocchio::Model::Index mEENode;
  int mNumControlledDofs;

  std::unique_ptr<ros::NodeHandle> mNodeHandle;

  std::vector<hardware_interface::JointHandle> mControlledJointHandles;

  // DYNAMIC PARAMETER OF KINOVA GEN3
  Eigen::MatrixXd mJointStiffnessMatrix;
  Eigen::MatrixXd mRotorInertiaMatrix;

  Eigen::MatrixXd mFrictionL;
  Eigen::MatrixXd mFrictionLp;

  Eigen::MatrixXd mJointKMatrix;
  Eigen::MatrixXd mJointDMatrix;

  long long int mCount;

  Eigen::VectorXd mActualTheta;
  Eigen::VectorXd mActualThetaDot;

  Eigen::VectorXd mDesiredPosition;
  Eigen::VectorXd mDesiredVelocity;
  Eigen::VectorXd mDesiredTheta;
  Eigen::VectorXd mDesiredThetaDot;

  Eigen::VectorXd mDesiredEffort;
  Eigen::VectorXd mJointEffort;

  Eigen::VectorXd mLastDesiredPosition;
  Eigen::VectorXd mLastDesiredVelocity;

  Eigen::VectorXd mNominalTheta;
  Eigen::VectorXd mNominalThetaDot;
  Eigen::VectorXd mNominalThetaDDot;

  Eigen::VectorXd mNominalThetaPrev;
  Eigen::VectorXd mNominalThetaDotPrev;

  Eigen::VectorXd mZeros;
  Eigen::VectorXd mGravity;
  Eigen::VectorXd mQuasiGravity;
  Eigen::VectorXd mNominalFriction;

  Eigen::VectorXd mActualPosition;
  Eigen::VectorXd mActualVelocity;
  Eigen::VectorXd mActualEffort;

  ExtendedJointPosition* mExtendedJoints;

  ros::Subscriber mSubCommand;
  void commandCallback(
      const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

  // for debugging
  std::chrono::time_point<std::chrono::high_resolution_clock> mLastTimePoint;

  // dynamic reconfigure
  dynamic_reconfigure::Server<
      gen3_compliant_controllers::JointSpaceCompliantControllerConfig>
      server;
  dynamic_reconfigure::Server<
      gen3_compliant_controllers::JointSpaceCompliantControllerConfig>::
      CallbackType f;

  void dynamicReconfigureCallback(
      gen3_compliant_controllers::JointSpaceCompliantControllerConfig& config,
      uint32_t level);
};

} // namespace gen3_compliant_controllers

#endif // GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H
