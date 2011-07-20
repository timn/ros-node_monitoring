
/***************************************************************************
 *  subscriber.cpp - Example: Node monitoring state subscriber
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
#include <nodemon_msgs/NodeState.h>

#include <cstdio>

void node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg)
{
  printf(".");
  fflush(stdout);
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nodemon_subex");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/nodemon/state", 10, node_state_cb);

  ros::spin();
  printf("\n");
}
