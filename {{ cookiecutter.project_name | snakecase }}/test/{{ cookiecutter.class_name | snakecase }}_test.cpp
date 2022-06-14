/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) {% now 'utc', '%Y' %}, PickNik Inc
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
 *   * Neither the name of PickNik Inc nor the names of its
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

/* Author: {{ cookiecutter.author }}
   Desc: TODO(GITHUB_NAME):
*/

/** EXAMPLES:
    EXPECT_FALSE(robot_state.hasFixedLinks());
    EXPECT_EQ(robot_state.getFixedLinksCount(), 0);
    EXPECT_TRUE(robot_state.getPrimaryFixedLink() == NULL);
    EXPECT_GT(robot_state.getFixedLinksMode(), 0);
    EXPECT_LT( fabs(vars[0] - 0), EPSILON) << "Virtual joint in wrong position "
   << vars[0];
*/

// C++
#include <string>

// Testing
#include <gtest/gtest.h>

// ROS
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

// Include class
#include <{{ cookiecutter.project_name | snakecase }}/{{ cookiecutter.class_name | snakecase }}.hpp>

class {{ cookiecutter.class_name}}Test : public ::testing::Test
{
public:
  {{ cookiecutter.class_name}}Test() { node_ = std::make_shared<{{ cookiecutter.project_name | snakecase }}::{{ cookiecutter.class_name}}>(); }

protected:
  rclcpp::Node::SharedPtr node_;
};  // class {{ cookiecutter.class_name}}Test

TEST_F({{ cookiecutter.class_name}}Test, NameOfTest)
{
  bool test = true;
  EXPECT_TRUE(test);
}

TEST_F({{ cookiecutter.class_name}}Test, SpinOnce)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.spin_some(std::chrono::nanoseconds(100));
  bool test = true;
  EXPECT_TRUE(test);
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  RCLCPP_INFO(rclcpp::get_logger("Test result"), "%d", result);

  rclcpp::shutdown();
  return 0;
}
