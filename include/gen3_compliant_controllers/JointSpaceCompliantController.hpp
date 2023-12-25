#ifndef GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H
#define GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H

// pinocchio headers
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/parsers/urdf.hpp>

// std headers
#include <chrono>
#include <string>
#include <vector>
using namespace std::chrono;

// ros headers
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// local headers
#include <gen3_compliant_controllers/JointSpaceCompliantControllerConfig.h>
#include <gen3_compliant_controllers/helpers.hpp>

namespace gen3_compliant_controllers {
class JointSpaceCompliantController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::JointStateInterface>
{
public:
  // Constructor and Destructor
  JointSpaceCompliantController();
  ~JointSpaceCompliantController();

  // Initialize the controller
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  // Lifecycle hooks for the controller
  void starting(const ros::Time& time) override; // Called before the first update
  void stopping(const ros::Time& time) override; // Called when the controller is stopped

  // Main update loop where control commands are computed and sent to the robot
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  // STATE AND COMMAND HANDLING
  std::unique_ptr<JointStateUpdater> mJointStateUpdater;                                 // Joint state updater
  bool shouldAcceptRequests();                                                           // Check if controller should accept requests
  bool shouldStopExecution(std::string& message);                                        // Check if controller should stop execution
  realtime_tools::RealtimeBuffer<trajectory_msgs::JointTrajectoryPoint> mCommandsBuffer; // Command buffer
  std::atomic_bool mExecuteDefaultCommand;                                               // Execute default command flag
  ros::Subscriber mSubCommand;                                                           // Subscriber for command messages
  void commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);        // Callback for command messages

  // CONTROLLER CONFIGURATION
  std::string mName;                                                    // Controller name
  std::vector<std::string> mJointNames;                                 // Controlled joint names
  std::unique_ptr<ros::NodeHandle> mNodeHandle;                         // Node handle
  std::vector<hardware_interface::JointHandle> mControlledJointHandles; // Joint handles

  // PINOCCHIO OBJECTS
  std::shared_ptr<pinocchio::Model> mModel; // Model object
  std::unique_ptr<pinocchio::Data> mData;   // Data object
  pinocchio::Model::Index mEENode;          // End effector node index
  int mNumControlledDofs;                   // Number of controlled joints

  // TUNABLE PARAMETERS
  Eigen::MatrixXd mJointStiffnessMatrix; // Joint stiffness matrix
  Eigen::MatrixXd mRotorInertiaMatrix;   // Rotor inertia matrix
  Eigen::MatrixXd mFrictionL;            // Friction observer matrix 1
  Eigen::MatrixXd mFrictionLp;           // Friction observer matrix 2
  Eigen::MatrixXd mJointKMatrix;         // Joint compliance proportional gain matrix
  Eigen::MatrixXd mJointDMatrix;         // Joint compliance derivative gain matrix

  long long int mCount; // Used during initialization

  // SENSOR READINGS
  Eigen::VectorXd mCurrentPosition; // Joint position read from sensor
  Eigen::VectorXd mCurrentVelocity; // Joint velocity read from sensor
  Eigen::VectorXd mCurrentEffort;   // Joint effort read from sensor

  // COMPUTED VARIABLES
  Eigen::VectorXd mCurrentTheta; // Normalized joint position

  Eigen::VectorXd mDesiredPosition; // Desired joint position
  Eigen::VectorXd mDesiredVelocity; // Desired joint velocity
  Eigen::VectorXd mDesiredTheta;    // Desired motor position
  Eigen::VectorXd mDesiredThetaDot; // Desired motor velocity

  Eigen::VectorXd mCommandEffort; // Commanded joint effort
  Eigen::VectorXd mTaskEffort;    // Task effort

  Eigen::VectorXd mLastDesiredPosition; // Last desired joint position (used for checking if desired position has changed)
  Eigen::VectorXd mLastDesiredVelocity; // Last desired joint velocity

  Eigen::VectorXd mNominalTheta;     // Nominal joint position computed from model
  Eigen::VectorXd mNominalThetaDot;  // Nominal joint velocity computed from model
  Eigen::VectorXd mNominalThetaDDot; // Nominal joint acceleration computed from model

  Eigen::VectorXd mNominalThetaPrev;    // Previous nominal joint position
  Eigen::VectorXd mNominalThetaDotPrev; // Previous nominal joint velocity

  Eigen::VectorXd mZeros;           // Vector of zeros
  Eigen::VectorXd mGravity;         // Gravity computed from model
  Eigen::VectorXd mQuasiGravity;    // Quasistatic gravity computed from model
  Eigen::VectorXd mNominalFriction; // Nominal friction computed from model

  ExtendedJointPosition* mExtendedJoints; // Used for computing normalized joint position

  // DEBUGGING
  std::chrono::time_point<std::chrono::high_resolution_clock> mLastTimePoint; 

  // DYNAMIC RECONFIGURE
  dynamic_reconfigure::Server<gen3_compliant_controllers::JointSpaceCompliantControllerConfig> server; // Dynamic reconfigure server
  dynamic_reconfigure::Server<gen3_compliant_controllers::JointSpaceCompliantControllerConfig>::CallbackType f; // Dynamic reconfigure callback type
  void dynamicReconfigureCallback(gen3_compliant_controllers::JointSpaceCompliantControllerConfig& config, uint32_t level); // Dynamic reconfigure callback function
};

} // namespace gen3_compliant_controllers

#endif // GEN3_COMPLIANT_CONTROLLERS__JOINT_SPACE_COMPLIANT_CONTROLLER_H
