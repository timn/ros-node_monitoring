
/***************************************************************************
 *  nodemon_thread.h - Thread to monitor a single node
 *
 *  Created: Sat Apr 09 12:11:54 2011
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

#ifndef __NODEMON_TUI_NODEMON_THREAD_H_
#define __NODEMON_TUI_NODEMON_THREAD_H_

#include <boost/thread.hpp>
#include <string>

class NodeMonThread : boost::thread
{
 public:
  NodeMonThread(std::string nodename);

  void heartbeat(const nodemon_msgs::NodeState::ConstPtr &msg);
  void run();
};


#endif
