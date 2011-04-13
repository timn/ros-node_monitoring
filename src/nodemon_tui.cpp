
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

NodeMonTUI::NodeMonTUI(ros::NodeHandle &nh)
  : __nh(nh)
{
  __wnd_width = __wnd_height = 0;
  __node_width = MIN_NODE_WIDTH;
  __win_msgs = NULL;

  __state_sub = __nh.subscribe("/nodemon/state", 10,
			       &NodeMonTUI::node_state_cb, this);
  __update_timer = __nh.createWallTimer(ros::WallDuration(0.2),
					&NodeMonTUI::update_timer_cb, this);
  initscr();   // Start curses mode
  curs_set(0); // invisible cursor
  noecho();    // disable printing of typed characters
  cbreak();    // disable line buffering
  timeout(0);  // make getch return immediately

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
  init_pair(CPAIR_ORANGE_BG, COLOR_BLACK, COLOR_YELLOW);
  init_pair(CPAIR_BLACK, COLOR_BLACK, -1);
  init_pair(CPAIR_BLACK_BG, COLOR_WHITE, COLOR_BLACK);

  reset_screen();
}


NodeMonTUI::~NodeMonTUI()
{
  delwin(__win_msgs);
  endwin(); // End curses mode
}


void
NodeMonTUI::print_debug(const char *str)
{
  int w, h;
  getmaxyx(stdscr, h, w);
  mvaddstr(h-1, 3, str);
}


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
    mvaddch(h-8, 0, ACS_LTEE);
    move(h - 8, 1);
    hline(0, w-2);
    mvaddch(h-8, w-1, ACS_RTEE);
    mvaddstr(0, 2, " Nodes ");
    mvaddstr(h-8, 2, " Messages ");
    attroff(A_BOLD);

    if (__win_msgs)  delwin(__win_msgs);
    __win_msgs = newwin(6, w-2, h-7, 1);
    scrollok(__win_msgs, TRUE);
  }
}

void
NodeMonTUI::update_screen()
{
  for (InfoMap::iterator i = __ninfo.begin(); i != __ninfo.end(); ++i) {
    chtype   cl = 0;
    const wchar_t *cc = L" ";
    int attrs = 0;

    if ((ros::WallTime::now() - i->second.last_update).toSec() > TIMEOUT) {
      cl = CPAIR_BLACK;
      attrs |= A_BOLD;
      // 231b  hourglass, unsupported in common fonts
      cc = L"\u231a";
    } else {
      switch (i->second.last_msg->state) {
      case nodemon_msgs::NodeState::STARTING:
	cl = CPAIR_BLACK_BG;
	attrs |= A_DIM;
	break;
      case nodemon_msgs::NodeState::RECOVERING: cl = CPAIR_ORANGE;       break;
      case nodemon_msgs::NodeState::ERROR:      cl = CPAIR_ORANGE_BG;    break;
      case nodemon_msgs::NodeState::FATAL:
	cl = CPAIR_RED;
	cc = L"\u2620";
	break;

      case nodemon_msgs::NodeState::STOPPING:
	cl = CPAIR_BLACK;
	attrs |= A_BOLD;
	break;
      default:
	break;
      }
    }
    if (i->second.updates == 0) {
      if (i->second.last_msg->state == nodemon_msgs::NodeState::FATAL) {
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
    i->second.updates += 1;
  }
  refresh(); // Print it on to the real screen
}


void
NodeMonTUI::print_messages()
{
  int y = 0;
  std::list<message_t>::iterator m;
  for (m = messages.begin(); m != messages.end(); ++m) {
    chtype   cl = 0;
    int attrs = 0;
    switch (m->state) {
    case nodemon_msgs::NodeState::ERROR:
      cl = CPAIR_ORANGE;
      break;
    case nodemon_msgs::NodeState::RECOVERING:
      cl = CPAIR_ORANGE;
      attrs |= A_BOLD;
      break;
    default:
      cl = CPAIR_RED;
      break;
    }
    attrs |= COLOR_PAIR(cl);
    wattron(__win_msgs, attrs);
    mvwaddstr(__win_msgs, y++, 0, m->message.c_str());
    wattroff(__win_msgs, attrs);
  }
  wrefresh(__win_msgs);
}

void
NodeMonTUI::reorder()
{
  int x = NODE_START_X;
  int y = NODE_START_Y;

  unsigned int line_entries = 0;

  for (InfoMap::iterator i = __ninfo.begin(); i != __ninfo.end(); ++i) {
    i->second.x = x;
    i->second.y = y;

    if (++line_entries >= NODES_PER_LINE) {
      line_entries = 0;
      x  = NODE_START_X;
      y += 1;
    } else {
      x += __node_width;
    }
  }
}


void
NodeMonTUI::read_key()
{
  int key = getch();
  if (key == ERR) return;

  // key code 27 is the Esc key
  if (((key == 27) && (getch() == ERR)) || (key == 'q')) {
    ros::shutdown();
  } else if (key == 'c') {
    clear();
  } else if (key == KEY_UP) {
    wscrl(__win_msgs,  1);
  } else if (key == KEY_DOWN) {
    wscrl(__win_msgs, -1);
  }
}

void
NodeMonTUI::add_node(std::string nodename)
{
  node_info_t info;
  info.last_update = ros::WallTime::now();
  info.updates = 0;
  info.x = 0;
  info.y = 0;
  __ninfo[nodename] = info;

  if ((nodename.length() + 2) > __node_width) {
    __node_width = nodename.length() + 2;
    reset_screen(/* force */ true);
  }

  reorder();
}


void
NodeMonTUI::clear()
{
  __ninfo.clear();
  messages.clear();
  __node_width = MIN_NODE_WIDTH;
  erase();
  reset_screen(/* force */ true);
  update_screen();
  print_messages();
}

void
NodeMonTUI::update_timer_cb(const ros::WallTimerEvent& event)
{
  reset_screen();
  update_screen();
  print_messages();
  read_key();
}

void
NodeMonTUI::node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg)
{
  if (__ninfo.find(msg->nodename) == __ninfo.end()) {
    add_node(msg->nodename);
  }
  __ninfo[msg->nodename].updates     = 0;
  __ninfo[msg->nodename].last_update =
    ros::WallTime(msg->time.sec, msg->time.nsec);
  __ninfo[msg->nodename].last_msg    = msg;

  if ((msg->state == nodemon_msgs::NodeState::ERROR) ||
      (msg->state == nodemon_msgs::NodeState::FATAL) ||
      (msg->state == nodemon_msgs::NodeState::RECOVERING))
  {
    if (msg->message != "") {
      messages.push_back(msg->message);
      if (messages.size() > 100) {
	messages.pop_front();
      }
    }
  }
}
