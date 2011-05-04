
/***************************************************************************
 *  state_publisher.h - Process monitoring state publisher
 *
 *  Created: Fri Apr 08 11:27:24 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *             2011  SRI International
 *             2011  Carnegie Mellon University
 *             2011  Intel Labs Pittsburgh
 *             2011  Columbia University in the City of New York
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the 3-clase BSD License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE file in the root directory.
 */

#include <ros/ros.h>
#include <nodemon_msgs/NodeState.h>

#ifndef __NODEMON_CPP_STATE_PUBLISHER_H_
#define __NODEMON_CPP_STATE_PUBLISHER_H_

class NodeStatePublisher
{
 public:
  NodeStatePublisher(const char *package_name, const char *node_type,
		     ros::NodeHandle &nh);
  ~NodeStatePublisher();

  void set_running();
  void set_fatal(std::string machine_msg, std::string human_msg);
  void set_error(std::string machine_msg, std::string msg);
  void set_recovering(std::string machine_msg, std::string msg);

  void send_warning(std::string machine_msg, std::string human_msg);

  void vset_fatal(const char *machine_msg, const char *human_format, ...);
  void vset_error(const char *machine_msg, const char *human_format, ...);
  void vset_recovering(const char *machine_msg, const char *human_format, ...);

  void vsend_warning(const char *machine_msg, const char *human_format, ...);

  void heartbeat_timer_cb(const ros::WallTimerEvent& event);

 private:
  inline void publish_state();

 private:
  ros::NodeHandle &__nh;

  ros::WallTimer __heartbeat_timer;

  ros::Publisher __state_pub;
  nodemon_msgs::NodeState __state_msg;
};

#endif
