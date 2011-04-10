
/***************************************************************************
 *  main.cpp - Node monitoring text-based user interface: main app
 *
 *  Created: Sat Apr 09 11:03:57 2011
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

#include "nodemon_tui.h"

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nodemon_subex");
  ros::NodeHandle n;

  if (!setlocale(LC_CTYPE, "")) {
    fprintf(stderr, "Can't set the specified locale! "
	    "Check LANG, LC_CTYPE, LC_ALL.\n");
    return 1;
  }

  NodeMonTUI tui(n);
  //printf("Test: %lc\n", L'\u2661');

  ros::spin(); // run ROS node main loop
  return 0;
}
