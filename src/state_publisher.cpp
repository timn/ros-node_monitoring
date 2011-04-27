
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

/** @class NodeStatePublisher <nodemon_cpp/state_publisher.h>
 * Node state publisher.
 * This class provides an interface to send heartbeat messages and specific
 * error information.
 *
 * To use this class, simply instantiate the class in your code with the
 * node handle. As soon as set_running() is called, the state publisher starts
 * to send periodical heartbeat messages. To switch the state, call any of
 * set_fatal(), set_error(), or set_recovering() for the respective state.
 * Call set_running() again to return to the normal operation state. Only in
 * this state the timestamp of the message is updated automatically. If you
 * want to indicate aliveness, for example during recovering, call the
 * respective set methods by yourself.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param nh node handle to register with
 */
NodeStatePublisher::NodeStatePublisher(ros::NodeHandle &nh)
  : __nh(nh)
{
  __state_pub = __nh.advertise<nodemon_msgs::NodeState>("/nodemon/state", 1);

  __heartbeat_timer = __nh.createWallTimer(ros::WallDuration(1.0),
					   &NodeStatePublisher::heartbeat_timer_cb,
					   this);

  __state_msg.nodename         = ros::this_node::getName();
  __state_msg.state            = nodemon_msgs::NodeState::STARTING;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = "";
  __state_msg.human_message    = "";
}


/** Destructor. */
NodeStatePublisher::~NodeStatePublisher()
{
  __state_msg.state            = nodemon_msgs::NodeState::STOPPING;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = "";
  __state_msg.human_message    = "";
  publish_state();
}


/** Publish the current state message. */
inline void
NodeStatePublisher::publish_state()
{
  __state_pub.publish(__state_msg);
}


/** Set the running state.
 * The state publisher will start to send periodical heartbeat messages
 * with updated time stamps.
 */
void
NodeStatePublisher::set_running()
{
  __state_msg.state            = nodemon_msgs::NodeState::RUNNING;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = "";
  __state_msg.human_message    = "";
  publish_state();
}


/** Set the fatal state.
 * The fatal state is meant for errors from which the node cannot recover
 * without completely restarting it. The message should give a meaningful and
 * concise description of the cause of the error.
 * @param machine_msg message describing the cause of the fatal error in a
 * machine parseable format
 * @param machine_msg message describing the cause of the fatal error in a
 * human readable format
 */
void
NodeStatePublisher::set_fatal(std::string machine_msg,
			      std::string human_msg)
{
  __state_msg.state            = nodemon_msgs::NodeState::FATAL;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = machine_msg;
  __state_msg.human_message    = human_msg;
  publish_state();
}


/** Set the non-fatal error state.
 * The error state is meant for errors from which the node can recover at
 * run-time. The node may require input from other nodes to start recovering,
 * or it can start recovery by itself if possible and without risk of damaging
 * the robot or harming humans. Once recovery is started, call set_recovering().
 * The message should give a meaningful and concise description of the cause
 * of the error.
 * @param machine_msg message describing the cause of the error in a
 * machine parseable format
 * @param machine_msg message describing the cause of the error in a
 * human readable format
 */
void
NodeStatePublisher::set_error(std::string machine_msg,
			      std::string human_msg)
{
  __state_msg.state            = nodemon_msgs::NodeState::ERROR;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = machine_msg;
  __state_msg.human_message    = human_msg;
  publish_state();
}


/** Set recovering state.
 * The recovery state is meant to describe that the node is in the process
 * of returning to an operational state. During that time it cannot process
 * any new requests or commands. Once recovery is finished call set_running()
 * to indicate that the node is fully operational again.
 * @param machine_msg a message describing the recovery method or procedure
 * briefly in a machine parseable format, e.g. "move_arm_safe_pos".
 * @param human_msg a message describing the recovery method or procedure
 * briefly in a human readable format, e.g. "moving arm to safe position".
 */
void
NodeStatePublisher::set_recovering(std::string machine_msg,
				   std::string human_msg)
{
  __state_msg.state            = nodemon_msgs::NodeState::RECOVERING;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = machine_msg;
  __state_msg.human_message    = human_msg;
  publish_state();
}


/** Send warning message.
 * Warning messages are meant to indicate a serious problem to the developer
 * or user, that the node was able to work around or solve this time, but
 * that might also cause trouble. The warning message is modeled as a
 * transitional state, i.e. an automatic transition is made back to the
 * previous state after the warning has been processed.
 * @param machine_msg a warning message describing the problem
 * briefly in a machine parseable format
 * @param human_msg a warning message describing the problem
 * briefly in a human readable format
 */
void
NodeStatePublisher::send_warning(std::string machine_msg,
				 std::string human_msg)
{
  nodemon_msgs::NodeState old_state = __state_msg;
  __state_msg.state            = nodemon_msgs::NodeState::WARNING;
  __state_msg.time             = ros::Time::now();
  __state_msg.machine_message  = machine_msg;
  __state_msg.human_message    = human_msg;
  publish_state();
  __state_msg = old_state;
  publish_state();
}

/** Callback for the heartbeat timer event.
 * @param event event description
 */
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
