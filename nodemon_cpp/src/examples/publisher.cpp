

/***************************************************************************
 *  publisher.cpp - Example: Node monitoring state publisher
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

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "nodemon_pubex");
  ros::NodeHandle n;

  NodeStatePublisher sp("nodemon_cpp", argv[0], n);
  sp.set_running();

  if (argc > 1) {
    if (strcmp(argv[1], "ERROR") == 0) {
      sp.set_error("broken", "Broken");
    } else if (strcmp(argv[1], "FATAL") == 0) {
      sp.set_fatal("broken", "Broken");
    }
  }

  ros::spin();
}
