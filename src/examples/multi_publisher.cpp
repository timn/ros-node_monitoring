

/***************************************************************************
 *  multi_publisher.cpp - Example: publish multiple "nodes"
 *
 *  Created: Fri Apr 08 12:16:30 2011
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
#include <nodemon/state_publisher.h>


ros::Publisher g_pub;
std::map<std::string, nodemon_msgs::NodeState> g_nodes;

unsigned int msgnum = 0;

void timer_cb(const ros::WallTimerEvent& event)
{
  std::map<std::string, nodemon_msgs::NodeState>::iterator p;
  for (p = g_nodes.begin(); p != g_nodes.end(); ++p) {
    nodemon_msgs::NodeState &msg = p->second;
    if (msg.state == nodemon_msgs::NodeState::RUNNING) {
      msg.time = ros::Time(event.current_real.sec, event.current_real.nsec);
    }
    msg.machine_message = msg.human_message = "";

    if (msg.state == nodemon_msgs::NodeState::FATAL) {
      msg.machine_message = msg.human_message = "FATAL";
    } else if (msg.state == nodemon_msgs::NodeState::ERROR) {
      msg.machine_message  = msg.human_message = "ERROR";
    } else if (msg.state == nodemon_msgs::NodeState::RECOVERING) {
      msg.machine_message  = msg.human_message = "RECOVERING";
    } else if (msg.state == nodemon_msgs::NodeState::WARNING) {
      msg.machine_message  = msg.human_message = "WARNING";
    }

    g_pub.publish(msg);
  }
}


void
add_fake_node(std::string nodename, uint8_t state)
{
  nodemon_msgs::NodeState msg;
  msg.nodename = nodename;
  msg.state = state;
  msg.time = ros::Time::now();

  g_nodes[nodename] = msg;
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nodemon_pubex");
  ros::NodeHandle n;

  g_pub = n.advertise<nodemon_msgs::NodeState>("/nodemon/state", 10);

  ros::WallTimer t = n.createWallTimer(ros::WallDuration(1.0), timer_cb);

  add_fake_node("/test1", nodemon_msgs::NodeState::STARTING);
  add_fake_node("/test2", nodemon_msgs::NodeState::RUNNING);
  add_fake_node("/test3", nodemon_msgs::NodeState::RECOVERING);
  add_fake_node("/test4", nodemon_msgs::NodeState::ERROR);
  add_fake_node("/test5", nodemon_msgs::NodeState::FATAL);
  add_fake_node("/test6", nodemon_msgs::NodeState::RUNNING);
  add_fake_node("/test7", nodemon_msgs::NodeState::RUNNING);
  add_fake_node("/test8", nodemon_msgs::NodeState::RUNNING);
  add_fake_node("/test8", nodemon_msgs::NodeState::WARNING);
  add_fake_node("/test9", nodemon_msgs::NodeState::STOPPING);

  ros::spin();
}
