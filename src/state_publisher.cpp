

/***************************************************************************
 *  state_publisher.cpp - Node monitoring state publisher
 *
 *  Created: Fri Apr 08 11:37:52 2011
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

#include <nodemon/state_publisher.h>

NodeStatePublisher::NodeStatePublisher(ros::NodeHandle &nh)
  : __nh(nh)
{
  __state_pub = __nh.advertise<nodemon_msgs::NodeState>("/nodemon/state", 1);

  __heartbeat_timer = __nh.createWallTimer(ros::WallDuration(1.0),
					   &NodeStatePublisher::heartbeat_timer_cb,
					   this);

  __state_msg.nodename = ros::this_node::getName();
  __state_msg.state    = nodemon_msgs::NodeState::STARTING;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = "";
}


NodeStatePublisher::~NodeStatePublisher()
{
  __state_msg.state    = nodemon_msgs::NodeState::STOPPING;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = "";
  publish_state();
}


inline void
NodeStatePublisher::publish_state()
{
  __state_pub.publish(__state_msg);
}

void
NodeStatePublisher::set_running()
{
  __state_msg.state    = nodemon_msgs::NodeState::RUNNING;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = "";
  publish_state();
}


void
NodeStatePublisher::set_fatal(std::string msg)
{
  __state_msg.state    = nodemon_msgs::NodeState::FATAL;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = msg;
  publish_state();
}


void
NodeStatePublisher::set_error(std::string msg)
{
  __state_msg.state    = nodemon_msgs::NodeState::ERROR;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = msg;
  publish_state();
}


void
NodeStatePublisher::set_recovering(std::string msg)
{
  __state_msg.state    = nodemon_msgs::NodeState::RECOVERING;
  __state_msg.time     = ros::Time::now();
  __state_msg.message  = msg;
  publish_state();
}


void
NodeStatePublisher::heartbeat_timer_cb(const ros::WallTimerEvent& event)
{
  if (__state_msg.state == nodemon_msgs::NodeState::RUNNING) {
    // we set the time automatically only when running!
    __state_msg.time =
      ros::Time(event.current_real.sec, event.current_real.nsec);
  }
  publish_state();  
}
