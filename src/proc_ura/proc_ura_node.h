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

#ifndef PROC_URA_NODE_H
#define PROC_URA_NODE_H

#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace proc_ura {

class ProcUraNode {
 public:
  //==========================================================================
  // P U B L I C   C / D T O R S

  ProcUraNode(const ros::NodeHandlePtr &nh);

  ~ProcUraNode();

  void Spin();
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_in);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  const double degree = M_PI/180;

  ros::NodeHandlePtr nh_;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_sub_;

  tf::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped odom_trans_;
  sensor_msgs::JointState joint_state_;

  double angle = 0;
};

}  // namespace proc_ura

#endif // PROC_URA_NODE_H
