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

// ROS
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <{{ cookiecutter.project_name | snakecase }}/{{ cookiecutter.class_name | snakecase }}.hpp>

int main(int argc, char ** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger("{{ cookiecutter.class_name | snakecase }}");
  RCLCPP_INFO(logger, "Starting {{ cookiecutter.class_name}}...");

  // Create a node.
  auto node = std::make_shared<{{ cookiecutter.project_name | snakecase }}::{{ cookiecutter.class_name}}>();

  /* Create an executor that will be responsible for execution of callbacks for
   * a set of nodes. With this version, all callbacks will be called from within
   * this thread (the main one). */
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  /* spin will block until work comes in, execute work as it becomes available,
   * and keep blocking. It will only be interrupted by Ctrl-C. */
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
