# Kinova Gen3 Compliant Controllers
![build](https://github.com/empriselab/gen3_compliant_controllers/actions/workflows/build-test.yml/badge.svg)

This repository provides joint-space and task-space compliant ROS1 controllers with [model-free friction observers](https://ieeexplore.ieee.org/document/8781838) for the Kinova Gen3 robot arm. The controller formulation implemented can be found [here](media/controller_formulation.pdf).

<p align="center">
  <img src="media/demo.gif" alt="Demo GIF" />
</p>

## Requirements

* `kortex_description` (for robot URDF files)
* `kortex_hardware` (ROS Hardware Interface for Kinova Gen3 arm)
* Pinocchio (for kinematics and inverse dynamics calculation)

## Installation

Assuming ROS1 is already installed, follow the **installation** and **usage** instructions [here](https://github.com/empriselab/kortex_hardware) to download and build `kortex_hardware`.

After following the previous instructions, you should have a catkin workspace. To build the controllers, run the following commands

```
cd ~/hw_test_ws/src
git clone https://github.com/empriselab/gen3_compliant_controllers
cd .. && catkin build
```

## Usage
It is preferred to have the robot and the system running ROS on the same network and connected via ethernet (especially important for torque control). For safety purposes, **make sure to have the e-stop next to you when testing.**

To ensure correct biasing of the robot's torque sensors, position the robot to the candlestick configuration with all joint positions set to zero. Then, zero out the actuator torque offset for each joint using the web application. For testing compliant control, return the robot to a general position like HOME; avoid testing at the candlestick position due to potential instability.

Running instructions for testing the controllers:
```
# In terminal 1
cd ~/hw_test_ws && source devel/setup.bash
roslaunch gen3_compliant_controllers controller.launch ip_address:=<robot-ip-address> dof:=<robot-dof> controller_type:=<joint/task> # see launch file for more parameters
# Robot IP should be the same as the one you use to access the web app.

# In terminal 2
cd ~/hw_test_ws && source devel/setup.bash
roscd gen3_compliant_controllers/scripts
python test_controllers.py joint # <task/joint>
# The robot will jerk a bit as it switches to effort mode

DOF: <specify joint number between 1-NDOF>
Value: <enter joint value> # in radians
# Ctrl + \ to exit
```

## Tuning Controller Parameters

Ideally, the joint stiffness matrix $K_j$, rotor inertia matrix $B$, friction-observer gains $L, L_p$, should only be tuned once for a given robot and remain fixed after that. Although, at any point if you observe vibrations, these parameters may require retuning. Beyond this, you may tune the PD gains of the motor-side controller to modulate the compliance behavior. This can also be done on-the-fly depending on your use case.

We provide support for tuning these parameters using [`dynamic_reconfigure`](http://wiki.ros.org/dynamic_reconfigure/Tutorials). The reconfigure node can also be accessed using a `dynamic_reconfigure` client ([tutorial](http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient)) to implement dynamic compliance behaviors.

To tune the parameters using the slider GUI, start the controller using the previously provided commands and open another terminal where you run 

```rosrun rqt_reconfigure rqt_reconfigure```  and select the `kortex_hardware` node to access the controller parameters.

Upon successfully tuning the parameters, dump the parameters using 

```rosparam get /kortex_hardware | grep -E '\b[bklj]|\b[d]_' | sed 's/^/kortex_hardware\//' > <path-to-gen3_compliant_controllers>/config/filename.yaml # regex ninja to get relevant parameters```.

These parameters can then be loaded into the ROS parameter server using the following snippet in your launch file.

```
<rosparam file="$(find gen3_compliant_controllers)/config/filename.yaml"/>
```

In case you would like to restart the tuning later with these parameters, use the GUI to save the YAML file and load it in the GUI when you start tuning again.

Refer to [this paper](https://ieeexplore.ieee.org/document/8781838) for a more conceptual understanding of the different controller parameters.

**Note**: Certain set of parameters may lead to severe vibrations. Please make sure you have the e-stop next to you or another person to manage the e-stop during the tuning process.


## Contribution
Any contributions related to bug fixes/feature additions/documentation are welcome! Contributing is simple. Just follow the following steps:
* Fork the repository
* Create a new branch with an appropriate name
* Make changes, test, and commit.
* Format the code using `catkin build gen3_compliant_controllers --no-deps --make-args format`
* Send a PR to the main repository and tag the maintainers to request a review.

## Authors
This implementation is done by [EmPRISE Lab](https://emprise.cs.cornell.edu/) at Cornell University in collaboration with [Intelligent Robotic Systems Lab](https://sites.google.com/view/kaist-roboticslab) at KAIST. It is currently maintained by Rishabh Madan ([@madan96](https://github.com/madan96)) and Rajat Kumar Jenamani ([@RKJenamani](https://github.com/RKJenamani)). It has received major initial contributions from [Seo Wook Han](mailto:tjdnr7117@kaist.ac.kr).
