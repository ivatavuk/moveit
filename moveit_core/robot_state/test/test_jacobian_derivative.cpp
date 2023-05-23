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
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace moveit::core;

namespace JDotTestHelpers
{
  Eigen::MatrixXd calculateJacobianDerivativeKDL(const std::vector<double> &q, 
                                                 const std::vector<double> &q_dot,
                                                 const RobotModel &robot_model) 
  {
    KDL::Chain kdl_chain;
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel( *(robot_model.getURDF()), tree) )
    {
      ROS_ERROR("Could not initialize tree object");
    }
    auto link_models = robot_model.getLinkModels();
    auto root_model = link_models[0]->getName();
    auto tip_model = link_models[q.size()]->getName();
    
    if (!tree.getChain(root_model, tip_model, kdl_chain))
    {
      ROS_ERROR_STREAM("Could not initialize chain object for base " << "a" << " tip " << "c");
    }

    KDL::JntArray kdl_jnt_array(q.size());
    kdl_jnt_array.data = Eigen::VectorXd::Map(q.data(), q.size());

    KDL::JntArray kdl_jnt_array_qdot(q_dot.size());
    kdl_jnt_array_qdot.data = Eigen::VectorXd::Map(q_dot.data(), q_dot.size());

    KDL::JntArrayVel kdl_jnt_array_vel;
    kdl_jnt_array_vel.q = kdl_jnt_array;
    kdl_jnt_array_vel.qdot = kdl_jnt_array_qdot;

    auto kdl_jacobian_dot_solver = KDL::ChainJntToJacDotSolver(kdl_chain);
    KDL::Jacobian kdl_jacobian_dot(kdl_chain.getNrOfJoints());

    kdl_jacobian_dot_solver.JntToJacDot(kdl_jnt_array_vel, kdl_jacobian_dot);

    return kdl_jacobian_dot.data;
  }
}

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
  std::cout << "Testing SimpleRobotJacobianDerivative!\n";

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd moveit_jacobian;
  Eigen::MatrixXd moveit_jacobian_derivative;
  bool use_quaternion_representation = false;
  auto joint_model_group = robot_model_->getJointModelGroup("group");

  //-----------------------Test for this state-----------------------
  std::vector<double> test_q{45.0 * M_PI / 180.0, 1.0};
  std::vector<double> test_qdot{0.1, 0.054};

  //-----------------------Set robot state-----------------------
  robot_state_->setJointGroupPositions(joint_model_group, 
                                       test_q);
  robot_state_->setJointGroupVelocities(joint_model_group, 
                                        test_qdot);
  robot_state_->updateLinkTransforms(); //TODO updateLinkTransforms() is needed, but it's not needed when first calling getJacobian?
  //-----------------------Calculate Jacobian in Moveit-----------------------
  /*robot_state_->getJacobian(joint_model_group,
                            robot_state_->getLinkModel("c"),
                            reference_point_position, moveit_jacobian,
                            use_quaternion_representation);
  */
  //-----------------------Calculate Jacobian Derivative in Moveit-----------------------
  robot_state_->getJacobianDerivative(joint_model_group,
                                      robot_state_->getLinkModel("c"),
                                      reference_point_position, moveit_jacobian_derivative,
                                      use_quaternion_representation);

  //-----------------------Calculate Jacobian Derivative with KDL-----------------------
  Eigen::MatrixXd kdl_jacobian_derivative = JDotTestHelpers::calculateJacobianDerivativeKDL(test_q, test_qdot, *robot_model_);

  //-----------------------Compare Jacobian Derivatives-----------------------
  std::cout << "Moveit Jacobian Derivative\n" << moveit_jacobian_derivative << "\n\n";
  std::cout << "KDL Jacobian Derivative\n" << kdl_jacobian_derivative << "\n\n";

  EXPECT_EIGEN_NEAR(moveit_jacobian_derivative, kdl_jacobian_derivative, 1e-5);
  //EXPECT_EQ(1.0, 1.0);
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
    robot_state_ = std::make_shared<RobotState>(robot_model_);
    ASSERT_TRUE(robot_state_->setToDefaultValues(jmg_, "ready"));
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
  RobotStatePtr robot_state_;
};
#ifndef _OLD_GTEST
RobotModelPtr PandaRobot::robot_model_;
JointModelGroup* PandaRobot::jmg_ = nullptr;
const LinkModel* PandaRobot::link_ = nullptr;
#endif

TEST_F(PandaRobot, testPandaRobotJacobianDerivative)
{
  ROS_WARN("Testing PandaRobotJacobianDerivative!");

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd moveit_jacobian;
  Eigen::MatrixXd moveit_jacobian_derivative;
  bool use_quaternion_representation = false;

  //-----------------------Test for this state-----------------------
  std::vector<double> test_q{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> test_qdot{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::generate(test_q.begin(), test_q.end(), []() {
    return (float) rand()/RAND_MAX;
  });

  std::generate(test_qdot.begin(), test_qdot.end(), []() {
    return (float) rand()/RAND_MAX;
  });

  //-----------------------Set robot state-----------------------
  robot_state_->setJointGroupPositions(jmg_, 
                                       test_q);
  robot_state_->setJointGroupVelocities(jmg_, 
                                        test_qdot);
  robot_state_->updateLinkTransforms(); //TODO updateLinkTransforms() is needed, but it's not needed when first calling getJacobian?
  //-----------------------Calculate Jacobian in Moveit-----------------------
  /*robot_state_->getJacobian(joint_model_group,
                            robot_state_->getLinkModel("c"),
                            reference_point_position, moveit_jacobian,
                            use_quaternion_representation);
  */
  //-----------------------Calculate Jacobian Derivative in Moveit-----------------------
  robot_state_->getJacobianDerivative(jmg_,
                                      link_,
                                      reference_point_position, moveit_jacobian_derivative,
                                      use_quaternion_representation);

  //-----------------------Calculate Jacobian Derivative with KDL-----------------------
  Eigen::MatrixXd kdl_jacobian_derivative = JDotTestHelpers::calculateJacobianDerivativeKDL(test_q, test_qdot, *robot_model_);

  //-----------------------Compare Jacobian Derivatives-----------------------
  std::cout << "Moveit Jacobian Derivative\n" << moveit_jacobian_derivative << "\n\n";
  std::cout << "KDL Jacobian Derivative\n" << kdl_jacobian_derivative << "\n\n";

  EXPECT_EQ(1.0, 1.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_jacobian_derivative");
  return RUN_ALL_TESTS();
}
