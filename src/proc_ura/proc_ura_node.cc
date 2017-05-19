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

#include "proc_ura_node.h"

namespace proc_ura {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcUraNode::ProcUraNode(const ros::NodeHandlePtr &nh) : nh_(nh) {
  joint_pub_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 1);
  joint_sub_ = nh_->subscribe<sensor_msgs::JointState>("/joint_states", 100, &ProcUraNode::JointStateCallback, this);

  odom_trans_.header.frame_id = "base_link";
  odom_trans_.child_frame_id = "base_plate";
}

//------------------------------------------------------------------------------
//
ProcUraNode::~ProcUraNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ProcUraNode::Spin() {
  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();

    //update joint_state
    joint_state_.header.stamp = ros::Time::now();
    joint_state_.name.resize(4);
    joint_state_.position.resize(4);
    joint_state_.name[0] ="base_rotation";
    joint_state_.name[1] ="bottom_rotation";
    joint_state_.name[2] ="top_rotation";
    joint_state_.name[3] ="tool_rotation";

//    odom_trans_.header.stamp = ros::Time::now();
//    odom_trans_.transform.translation.x = 0;
//    odom_trans_.transform.translation.y = 0;
//    odom_trans_.transform.translation.z = 0;
//    odom_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    //send the joint state and transform
    joint_pub_.publish(joint_state_);
//    broadcaster_.sendTransform(odom_trans_);

    angle += degree/4;

    r.sleep();
  }
}

void ProcUraNode::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_in) {
  joint_state_.position[0] = msg_in->position[0];
  joint_state_.position[1] = msg_in->position[1];
  joint_state_.position[2] = msg_in->position[2];
  joint_state_.position[3] = msg_in->position[3];
}

}  // namespace proc_ura
