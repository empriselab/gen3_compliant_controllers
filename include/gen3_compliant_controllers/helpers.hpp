#ifndef GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_
#define GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_

// pinocchio headers
#include <pinocchio/algorithm/model.hpp>

// std headers
#include <string>
#include <typeinfo>
#include <vector>

// ros headers
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

namespace gen3_compliant_controllers {

//=============================================================================
struct JointParameter
{
  std::string mName; // Name of the joint
  std::string mType; // Type of the joint
};

//=============================================================================
class JointStateUpdater final
{
public:
  JointStateUpdater(std::shared_ptr<pinocchio::Model> model, hardware_interface::JointStateInterface* jointStateInterface); // Constructor

  void update(); // Update the joint state

  Eigen::VectorXd mCurrentPosition; // Current position
  Eigen::VectorXd mCurrentVelocity; // Current velocity
  Eigen::VectorXd mCurrentEffort;   // Current effort

private:
  std::shared_ptr<pinocchio::Model> mModel; // Model of the robot

  Eigen::VectorXd mDefaultPosition; // Default position
  Eigen::VectorXd mDefaultVelocity; // Default velocity
  Eigen::VectorXd mDefaultEffort;   // Default effort

  std::vector<hardware_interface::JointStateHandle> mHandles; // Joint state handles
};

//=============================================================================
class ExtendedJointPosition
{
private:
  unsigned int mNumberOfInput; // DOFs of the robot
  double mThresholdOfChange;   // Threshold of change (Extended joint position is updated only if the change is greater than this threshold)

  Eigen::VectorXd mInitQ;           // Initial joint position
  Eigen::VectorXd mExtendedQ;       // Extended joint position
  Eigen::VectorXd mPreviousSensorQ; // Previous sensor joint position

public:
  ExtendedJointPosition(unsigned int number_of_input_args, double threshold_of_change_args); // Constructor
  void initializeExtendedJointPosition(const Eigen::VectorXd& init_q_args);                  // Initialize the extended joint position
  double normalizeJointPosition(double input);                                               // Normalize a joint position
  Eigen::VectorXd normalizeJointPosition(const Eigen::VectorXd& input);                      // Normalize a joint position vector
  void estimateExtendedJoint(const Eigen::VectorXd& current_sensor_q);                       // Estimate the extended joint position from the sensor joint position
  Eigen::VectorXd getExtendedJoint();                                                        // Get the extended joint position

  bool mIsInitialized = false;          // Initialization flag
  Eigen::VectorXd mLastDesiredPosition; // Last desired position

  Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat, double eps = 1e-6);                           // Compute the pseudoinverse of a matrix
  Eigen::MatrixXd computeEEMassMatrix(Eigen::MatrixXd mMassMatrix, Eigen::VectorXd q, Eigen::MatrixXd J); // Compute the end-effector mass matrix
};

//=============================================================================
Eigen::VectorXd joint_ros_to_pinocchio(Eigen::VectorXd& q, pinocchio::Model& model); // Convert a joint position from ROS config to Pinocchio config

//=============================================================================
std::vector<JointParameter> loadJointsFromParameter(const ros::NodeHandle& nodeHandle, const std::string& jointsParameter, const std::string& defaultType); // Load joints from ROS parameter server

} // namespace gen3_compliant_controllers

#endif // ifndef GEN3_COMPLIANT_CONTROLLERS_HELPERS_HPP_
