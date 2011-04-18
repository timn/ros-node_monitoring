
/***************************************************************************
 *  nodemon_tui.cpp - Node monitoring text-based user interface
 *
 *  Created: Fri Apr 08 12:57:45 2011
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

#include "nodemon_tui.h"

#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>

/** @class NodeMonTUI "nodemon_tui.h"
 * Node monitoring TUI.
 * This class implements a text-based user interface (TUI) using the ncurses
 * library. It shows heartbeat and error information about processes.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param nh ROS node handle
 */
NodeMonTUI::NodeMonTUI(ros::NodeHandle &nh)
  : __nh(nh)
{
  __wnd_width = __wnd_height = 0;
  __node_width = MIN_NODE_WIDTH;
  __win_msgs = NULL;
  __msg_start = 0;

  __state_sub = __nh.subscribe("/nodemon/state", 10,
			       &NodeMonTUI::node_state_cb, this);
  __update_timer = __nh.createWallTimer(ros::WallDuration(UPDATE_INTERVAL_SEC),
					&NodeMonTUI::update_timer_cb, this);

  initscr();   // Start curses mode
  curs_set(0); // invisible cursor
  noecho();    // disable printing of typed characters
  cbreak();    // disable line buffering
  timeout(0);  // make getch return immediately
  keypad(stdscr, TRUE);

  if(has_colors() == FALSE) {
    endwin();
    printf("You terminal does not support color\n");
    exit(1);
  }
  start_color();
  use_default_colors();   // initialize default colors
  // defining colors doesn't work in bash/gnome-terminal
  //init_color(COLOR_LIGHT_GREY, 750, 750, 750);
  //init_color(COLOR_DARK_GREY,  250, 250, 250);
  //init_color(COLOR_ORANGE,    1000, 500,   0);
  init_pair(CPAIR_RED, COLOR_RED, -1);
  init_pair(CPAIR_ORANGE, COLOR_YELLOW, -1);
  init_pair(CPAIR_MAGENTA, COLOR_MAGENTA, -1);
  init_pair(CPAIR_BLACK, COLOR_BLACK, -1);
  init_pair(CPAIR_BLACK_BG, COLOR_WHITE, COLOR_BLACK);

  reset_screen();

  __cache_path = "";
  // try to read cache
  const char *home = getenv("HOME");
  if (home != NULL) {
    __dot_ros_dir = std::string(home) + "/" DOT_ROS_DIR;
    __cache_path  = __dot_ros_dir + "/" NODEMON_CACHE_FILE;

    // just in case...
    mkdir(__dot_ros_dir.c_str(), 0700);

    FILE *f = fopen(__cache_path.c_str(), "r");
    if (f != NULL) {
      char tmp[1024];
      while (fgets(tmp, 1024, f) != NULL) {
	tmp[strlen(tmp) - 1] = '\0'; // remove newline
	add_node(tmp, /* add to cache */ false);
      }
      fclose(f);
    }
  }
}


/** Destructor. */
NodeMonTUI::~NodeMonTUI()
{
  delwin(__win_msgs);
  endwin(); // End curses mode
}


/** Print debug message.
 * The message will appear in the lower left corner of the main window.
 * Consecutive calls to this method will overwrite the previous message.
 * @param str string to print
 */
void
NodeMonTUI::print_debug(const char *str)
{
  mvaddstr(__wnd_height-1, 3, str);
}


/** Reset the screen.
 * If the screen size has changed, or the @p force flag is set to true,
 * the screen will be erased and the chrome (borders, titles) will be
 * drawn.
 * @param force true to force a re-draw, false to only redraw if screen
 * dimensions have changed.
 */
void
NodeMonTUI::reset_screen(bool force)
{
  int h, w;
  getmaxyx(stdscr, h, w);

  if (force) {
    __wnd_width = __wnd_height = -1;
  }

  if (w != __wnd_width || h != __wnd_height) {
    __wnd_width  = w;
    __wnd_height = h;

    erase();

    attron(A_BOLD);
    border(0, 0, 0, 0, 0, 0, 0, 0);
    mvaddch(h - (NUM_MSG_LINES + 2), 0, ACS_LTEE);
    move(h - (NUM_MSG_LINES + 2), 1);
    hline(0, w-2);
    mvaddch(h - (NUM_MSG_LINES + 2), w-1, ACS_RTEE);
    mvaddstr(0, 2, " Nodes ");
    mvaddstr(h - (NUM_MSG_LINES + 2), 2, " Messages ");
    attroff(A_BOLD);

    if (__win_msgs)  delwin(__win_msgs);
    __win_msgs = newwin(NUM_MSG_LINES, w - 2, h - (NUM_MSG_LINES+1), 1);
    scrollok(__win_msgs, TRUE);
  }
}

/** Update screen content.
 * This will print the current information about nodes.
 */
void
NodeMonTUI::update_screen()
{
  for (InfoMap::iterator i = __ninfo.begin(); i != __ninfo.end(); ++i) {
    chtype   cl = 0;
    const wchar_t *cc = L" ";
    int attrs = 0;

    ros::WallTime now = ros::WallTime::now();

    bool timed_out =
      ((now - i->second.last_update).toSec() > TIMEOUT_SEC);

    bool state_hang = i->second.last_msg &&
      ((ros::Time::now() - i->second.last_msg->time).toSec() > TIMEOUT_SEC);

    if (! i->second.last_msg) {
      // Node which has been added from cache, we're waiting for a heartbeat
      cl = CPAIR_BLACK;
      attrs |= A_BOLD;
      cc = L"\u2300";

    } else if (timed_out) {
      cl = CPAIR_BLACK;
      attrs |= A_BOLD;
      // 231b  hourglass, unsupported in common fonts
      cc = L"\u231a";

    } else {
      switch (i->second.last_msg->state) {
      case nodemon_msgs::NodeState::STARTING:
	attrs |= A_BOLD;
	break;

      case nodemon_msgs::NodeState::RECOVERING:
	cl = CPAIR_ORANGE;
	cc = L"\u26a0";
	break;

      case nodemon_msgs::NodeState::ERROR:
	cl = CPAIR_RED;
	cc = L"\u26a0";
	break;

      case nodemon_msgs::NodeState::FATAL:
	cl = CPAIR_RED;
	attrs |= A_BOLD;
	cc = L"\u2620";
	break;

      case nodemon_msgs::NodeState::STOPPING:
	cl = CPAIR_BLACK;
	attrs |= A_BOLD;
	break;
      default:
	break;
      }

      if (state_hang) {
	// 231b  hourglass, unsupported in common fonts
	cc = L"\u231a";
      }
    }
    if (i->second.last_msg &&
	((now - i->second.last_update).toSec() <= UPDATE_INTERVAL_SEC))
    {
      if ((i->second.last_msg->state == nodemon_msgs::NodeState::FATAL) ||
	  state_hang)
      {
	cc = L"\u2661";
      } else {
	cc = L"\u2665";
      }
    }

    attrs |= COLOR_PAIR(cl);
    attron(attrs);
    mvaddwstr(i->second.y, i->second.x, cc);
    mvaddstr(i->second.y, i->second.x + 2, i->first.c_str());
    attroff(attrs);
  }
  refresh(); // Print it on to the real screen
}


/** Print messages.
 * This prints the message strings.
 */
void
NodeMonTUI::print_messages()
{
  werase(__win_msgs);


  int y = 0;
  std::list<message_t>::size_type num = 0;
  std::list<message_t>::iterator m = __messages.begin();
  std::list<message_t>::size_type i;
  for (i = 0; m != __messages.end() && i < __msg_start; ++m, ++i) ;

  for (; m != __messages.end() && num < NUM_MSG_LINES; ++m, ++num) {
    chtype   cl = 0;
    int attrs = 0;
    switch (m->state) {
    case nodemon_msgs::NodeState::ERROR:
      cl = CPAIR_RED;
      break;
    case nodemon_msgs::NodeState::RECOVERING:
      cl = CPAIR_ORANGE;
      break;
    default:
      cl = CPAIR_RED;
      attrs |= A_BOLD;
      break;
    }
    attrs |= COLOR_PAIR(cl);
    wattron(__win_msgs, attrs);
    mvwaddstr(__win_msgs, y++, 0, m->message.c_str());
    wattroff(__win_msgs, attrs);
  }
  wrefresh(__win_msgs);
}


/** Reorder nodes.
 * This recalculates the positions for all nodes.
 */
void
NodeMonTUI::reorder()
{
  unsigned int x = NODE_START_X;
  unsigned int y = NODE_START_Y;

  for (InfoMap::iterator i = __ninfo.begin(); i != __ninfo.end(); ++i) {
    i->second.x = x;
    i->second.y = y;

    if ((x + (2 * __node_width)) > __wnd_width) {
      x  = NODE_START_X;
      y += 1;
    } else {
      x += __node_width;
    }
  }
}


/** Read a key and call the appropriate callbacks. */
void
NodeMonTUI::read_key()
{
  int key = getch();
  if (key == ERR) return;

  // key code 27 is the Esc key
  if (key == KEY_UP) {
    if (__msg_start > 0) {
      --__msg_start;
      print_messages();
      print_debug("UP");
    }
  } else if (key == KEY_DOWN) {
    if (__msg_start < __messages.size() - NUM_MSG_LINES) {
      ++__msg_start;
      print_messages();
    }
  } else if (((key == 27) && (getch() == ERR)) || (key == 'q')) {
    ros::shutdown();
  } else if (key == 'c') {
    clear();
  } else if (key == 'C') {
    clear_messages();
  }
}

/** Add a node.
 * @param nodename name of the node to add
 * @param add_to_cache true to add the node to the cache file, false not to
 */
void
NodeMonTUI::add_node(std::string nodename, bool add_to_cache)
{
  node_info_t info;
  info.last_update = ros::WallTime::now();
  info.x = 0;
  info.y = 0;
  __ninfo[nodename] = info;

  if ((nodename.length() + 6) > __node_width) {
    __node_width = nodename.length() + 6;
    reset_screen(/* force */ true);
  }

  reorder();

  if (add_to_cache && __cache_path != "") {
    FILE *f = fopen(__cache_path.c_str(), "a");
    if (f != NULL) {
      fprintf(f, "%s\n", nodename.c_str());
      fclose(f);
    }
  }
}


/** Clear the list of nodes and messages. */
void
NodeMonTUI::clear()
{
  __ninfo.clear();
  __messages.clear();

  if (__cache_path != "") {
    FILE *f = fopen(__cache_path.c_str(), "w");
    if (f != NULL) {
      fclose(f);
    }
  }

  __node_width = MIN_NODE_WIDTH;
  erase();
  reset_screen(/* force */ true);
  update_screen();
  print_messages();
}


/** Clear message list. */
void
NodeMonTUI::clear_messages()
{
  __messages.clear();
  erase();
  reset_screen(/* force */ true);
  update_screen();
  print_messages();
}

/** Callback for update timer event.
 * @param event event information
 */
void
NodeMonTUI::update_timer_cb(const ros::WallTimerEvent& event)
{
  reset_screen();
  update_screen();
  print_messages();
  read_key();
}

/** Callback when receiving a node state message.
 * @param msg received message.
 */
void
NodeMonTUI::node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg)
{
  if (__ninfo.find(msg->nodename) == __ninfo.end()) {
    add_node(msg->nodename);
  }

  ros::WallTime msg_walltime(msg->time.sec, msg->time.nsec);

  if (((msg->state == nodemon_msgs::NodeState::ERROR) ||
       (msg->state == nodemon_msgs::NodeState::FATAL) ||
       (msg->state == nodemon_msgs::NodeState::RECOVERING)) &&
      (msg->message != "") &&
      (!__ninfo[msg->nodename].last_msg ||
       ((__ninfo[msg->nodename].last_msg->time != msg->time) &&
	(__ninfo[msg->nodename].last_msg->message != msg->message))))
  {
    tm time_tm;
    time_t timet;
    timet = msg_walltime.sec;
    localtime_r(&timet, &time_tm);
           // stat  date  nodename              message              NULL
    size_t ml = 2 + 21 + msg->nodename.size() + msg->message.size() + 1;
    char mstr[ml];

    const char *state = "F";
    if (msg->state == nodemon_msgs::NodeState::ERROR) {
      state = "E";
    } else if (msg->state == nodemon_msgs::NodeState::RECOVERING) {
      state = "R";
    }

    sprintf(mstr, "%s %02d:%02d:%02d.%09u %s: %s", state, time_tm.tm_hour,
	    time_tm.tm_min, time_tm.tm_sec, msg_walltime.nsec,
	    msg->nodename.c_str(), msg->message.c_str());

    message_t m;
    m.state   = msg->state;
    m.message = mstr;

    __messages.push_front(m);
    __msg_start = 0;
    if (__messages.size() > MAX_NUM_MSGS) {
      __messages.pop_back();
    }
  }

  __ninfo[msg->nodename].last_update = ros::WallTime::now();
  __ninfo[msg->nodename].last_msg    = msg;
}
