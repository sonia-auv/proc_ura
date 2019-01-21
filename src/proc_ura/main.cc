/**
 * \file	main.cc
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	05/04/2017
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

//#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include "proc_ura/proc_ura_node.h"
//
//int main(int argc, char **argv) {
//  ros::init(argc, argv, "proc_ura");
//
//
//
////  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
////  proc_ura::ProcUraNode proc_ura_node{nh};
////  proc_ura_node.Spin();
//
//  return 0;
//}


#include <vector>
#include <string>
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

class JointStateAggregator
{

 public:
  JointStateAggregator()
  {
    private_nh_ = ros::NodeHandle("~");
    private_nh_.param<int>("rate", publish_rate_, 50);
  }

  bool initialize()
  {
    XmlRpc::XmlRpcValue val;
    std::vector<std::string> static_joints;

    if (private_nh_.getParam("static_joints", val))
    {
      if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("static_joints parameter is not a list");
        return false;
      }

      for (int i = 0; i < val.size(); ++i)
      {
        static_joints.push_back(static_cast<std::string>(val[i]));
      }
    }

    std::vector<std::string> controller_names;

    if (private_nh_.getParam("controllers", val))
    {
      if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("controllers parameter is not a list");
        return false;
      }

      for (int i = 0; i < val.size(); ++i)
      {
        controller_names.push_back(static_cast<std::string>(val[i]));
      }
    }

    int num_static = static_joints.size();
    int num_controllers = controller_names.size();
    int num_total = num_static + num_controllers;

    msg_.name.resize(num_total);
    msg_.position.resize(num_total);
    msg_.velocity.resize(num_total);
    msg_.effort.resize(num_total);

    for (int i = 0; i < num_static; ++i)
    {
      msg_.name[i] = static_joints[i];
      msg_.position[i] = 0.0;
      msg_.velocity[i] = 0.0;
      msg_.effort[i] = 0.0;
    }

    controller_state_subs_.resize(num_controllers);

    for (int i = 0; i < num_controllers; ++i)
    {
      controller_state_subs_[i] =
          nh_.subscribe<dynamixel_msgs::JointState>(controller_names[i] +  "/state", 100,
                                                    boost::bind(&JointStateAggregator::processControllerState, this, _1, i+num_static));
    }

    for (int i = 0; i < num_controllers; ++i)
    {
      ros::topic::waitForMessage<dynamixel_msgs::JointState>(controller_names[i] +  "/state");
    }

    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);

    return true;
  }

  void processControllerState(const dynamixel_msgs::JointStateConstPtr& msg, int i)
  {
    msg_.name[i] = msg->name;
    msg_.position[i] = msg->current_pos;
    msg_.velocity[i] = msg->velocity;
    msg_.effort[i] = msg->load;
  }

  void start()
  {
    ros::Rate r(publish_rate_);

    while (ros::ok())
    {
      msg_.header.stamp = ros::Time::now();
      joint_states_pub_.publish(msg_);

      ros::spinOnce();
      r.sleep();
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::V_Subscriber controller_state_subs_;
  ros::Publisher joint_states_pub_;

  int publish_rate_;
  sensor_msgs::JointState msg_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "proc_ura");
  JointStateAggregator jsa;

  if (!jsa.initialize())
  {
    ROS_ERROR("JointStateAggregator failed to initialize");
    return 1;
  }

  jsa.start();

  return 0;
}