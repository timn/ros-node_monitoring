
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

#define MIN_NODE_WIDTH 24
#define NODES_PER_LINE 3
#define NODE_START_X 2
#define NODE_START_Y 2

#define NUM_MSG_LINES 16
#define MAX_NUM_MSGS 100

#define UPDATE_INTERVAL_SEC 0.2
#define TIMEOUT_SEC 5.0

#define CPAIR_RED          1
#define CPAIR_ORANGE       2
#define CPAIR_BLACK        3
#define CPAIR_BLACK_BG     4
#define CPAIR_MAGENTA      5

#define DOT_ROS_DIR ".ros"
#define NODEMON_CACHE_FILE "nodemon_cache"

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
  void reset_screen(bool force = false);
  void update_screen();
  void print_messages();
  void print_debug(const char *str);
  void read_key();
  bool reorder();
  void add_node(std::string nodename, bool add_to_cache = true);
  void clear();
  void clear_messages();

 private:
  ros::NodeHandle &__nh;

  ros::Subscriber  __state_sub;
  ros::WallTimer   __update_timer;

  std::string __cache_path;
  std::string __dot_ros_dir;

  int __node_width;
  int __wnd_width;
  int __wnd_height;

  typedef struct {
    ros::WallTime  last_update;
    int  x;
    int  y;
    nodemon_msgs::NodeState::ConstPtr last_msg;
  } node_info_t;

  
  typedef std::map<std::string, node_info_t> InfoMap;
  InfoMap __ninfo;

  typedef struct {
    uint8_t state;
    std::string message;
  } message_t;
  std::list<message_t> __messages;
  std::list<message_t>::size_type __msg_start;
  

  WINDOW *__win_msgs;
};


#endif
