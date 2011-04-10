
/***************************************************************************
 *  nodemon_tui.h - Node monitoring text-based user interface
 *
 *  Created: Sat Apr 09 11:04:24 2011
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

#include <ncurses.h>
#include <list>
#include <map>
#include <string>

#define MIN_SCREEN_WIDTH  80
#define MIN_SCREEN_HEIGHT 25

#define NODE_WIDTH 24
#define NODES_PER_LINE 3
#define NODE_START_X 2
#define NODE_START_Y 2

#define TIMEOUT 5.0

#define CPAIR_RED          1
#define CPAIR_ORANGE       2
#define CPAIR_BLACK        3
#define CPAIR_BLACK_BG     4
#define CPAIR_ORANGE_BG    5

#ifndef __NODEMON_TUI_NODEMON_TUI_H_
#define __NODEMON_TUI_NODEMON_TUI_H_

class NodeMonTUI
{
 public:
  NodeMonTUI(ros::NodeHandle &nh);
  ~NodeMonTUI();

  void node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg);

  void update_timer_cb(const ros::WallTimerEvent& event);

 private:
  void update_screen();
  void print_messages();
  void print_debug(const char *str);
  void read_key();
  void reorder();
  void add_node(std::string nodename);

 private:
  ros::NodeHandle &__nh;

  ros::Subscriber  __state_sub;
  ros::WallTimer   __update_timer;

  typedef struct {
    ros::WallTime  last_update;
    int  updates;
    int  x;
    int  y;
    nodemon_msgs::NodeState::ConstPtr last_msg;
  } node_info_t;

  
  typedef std::map<std::string, node_info_t> InfoMap;
  InfoMap __ninfo;

  std::list<std::string> messages;

  WINDOW *__msgwin;
};


#endif
