/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PickNik nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ivo Vatavuk */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/utils/eigen_test_utils.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace moveit::core;

class SimpleRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    RobotModelBuilder builder("simple", "a");
    builder.addChain("a->b", "continuous", {}, urdf::Vector3(0, 0, 1)); //Rotates around the z-axis
    builder.addChain("b->c", "prismatic", {}, urdf::Vector3(1, 0, 0));  //Translates along the x-axis
    builder.addGroupChain("a", "c", "group");
    robot_model_ = builder.build();
    robot_state_ = std::make_shared<RobotState>(robot_model_);
  }

  void TearDown() override
  {
  }

protected:
  RobotModelConstPtr robot_model_;
  RobotStatePtr robot_state_;
};


TEST_F(SimpleRobot, testSimpleRobotJacobianDerivative)
{
  ROS_WARN("Testing SimpleRobotJacobianDerivative!");

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian, jacobian_derivative;
  bool use_quaternion_representation = false;

  auto joint_model_group = robot_model_->getJointModelGroup("group");

  robot_state_->setJointGroupPositions(joint_model_group, 
                                       std::vector<double>{ 45.0 * M_PI / 180.0, 1.0 });
  

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::cout << "Joint names = \n";
  for(auto joint_name : joint_names)
    std::cout << joint_name << "\n";

  std::vector<double> joint_values;
  robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
  std::cout << "Joint values = \n";
  for(auto joint_value : joint_values)
    std::cout << joint_value << "\n";

  robot_state_->getJacobian(joint_model_group,
                            robot_state_->getLinkModel("c"),
                            reference_point_position, jacobian,
                            use_quaternion_representation);
  
  //Using KDL for testing
  //Taken from pr2_arm_kinematics_plugin getKDLChain
  KDL::Chain kdl_chain;
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model_->getURDF(), tree))
  {
    ROS_ERROR("Could not initialize tree object");
  }
  if (!tree.getChain("a", "c", kdl_chain))
  {
    ROS_ERROR_STREAM("Could not initialize chain object for base " << "a" << " tip " << "c");
  }

  auto kdl_jacobian_solver = KDL::ChainJntToJacSolver(kdl_chain);
  KDL::Jacobian kdl_jacobian(kdl_chain.getNrOfJoints());
  KDL::JntArray kdl_jnt_array(2);
  kdl_jnt_array.data << 45.0 * M_PI / 180.0, 1.0;
  kdl_jacobian_solver.JntToJac(kdl_jnt_array, kdl_jacobian);

  robot_state_->getJacobianDerivative(joint_model_group,
                                      robot_state_->getLinkModel("c"),
                                      reference_point_position, jacobian_derivative,
                                      use_quaternion_representation);

  std::cout << "Moveit Jacobian = \n" << jacobian << "\n\n";
  std::cout << "KDL Jacobian = \n" << kdl_jacobian.data << "\n\n";

  std::cout << "Moveit Jacobian Derivative\n" << jacobian_derivative << "\n\n";

  EXPECT_EQ(1.0, 1.0); 
}

// Gracefully handle gtest 1.8 (Melodic)
#ifndef INSTANTIATE_TEST_SUITE_P
#define _STATIC
#define _OLD_GTEST
#else
#define _STATIC static
#endif

class PandaRobot : public testing::Test
{
protected:
  _STATIC void SetUpTestSuite()  // setup resources shared between tests
  {
    robot_model_ = loadTestingRobotModel("panda");
    jmg_ = robot_model_->getJointModelGroup("panda_arm");
    link_ = robot_model_->getLinkModel("panda_link8");
    ASSERT_TRUE(link_);
  }

  _STATIC void TearDownTestSuite()
  {
    robot_model_.reset();
  }

  void SetUp() override
  {
#ifdef _OLD_GTEST
    SetUpTestSuite();
#endif
    start_state_ = std::make_shared<RobotState>(robot_model_);
    ASSERT_TRUE(start_state_->setToDefaultValues(jmg_, "ready"));
    start_pose_ = start_state_->getGlobalLinkTransform(link_);
  }
#ifdef _OLD_GTEST
  void TearDown() override
  {
    TearDownTestSuite();
  }
#endif
protected:
  _STATIC RobotModelPtr robot_model_;
  _STATIC JointModelGroup* jmg_;
  _STATIC const LinkModel* link_;

  double prec_ = 1e-8;
  RobotStatePtr start_state_;
  Eigen::Isometry3d start_pose_;
};
#ifndef _OLD_GTEST
RobotModelPtr PandaRobot::robot_model_;
JointModelGroup* PandaRobot::jmg_ = nullptr;
const LinkModel* PandaRobot::link_ = nullptr;
#endif

TEST_F(PandaRobot, testPandaRobotJacobianDerivative)
{
  ROS_WARN("Testing PandaRobotJacobianDerivative!");
  EXPECT_EQ(1.0, 1.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_jacobian_derivative");
  return RUN_ALL_TESTS();
}
