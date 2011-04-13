#! /usr/bin/env python

#---------------------------------------------------------------------------
#  state_publisher.py - Node monitoring state publisher
#
#  Created: Thu Apr 13 11:42:21 2011
#  Copyright  2011  Tim Niemueller [www.niemueller.de]
#             2011  SRI International
#             2011  Carnegie Mellon University
#             2011  Intel Labs Pittsburgh
#             2011  Columbia University in the City of New York
#
#---------------------------------------------------------------------------


#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the 3-clase BSD License.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE file in the root directory.
#

import roslib; roslib.load_manifest('nodemon_py')
import rospy

from nodemon_msgs.msg import NodeState
from threading import Timer

class NodeStatePublisher(object):
    def __init__(self):
        self._state_pub = rospy.Publisher('/nodemon/state', NodeState);
        
        self._state_msg = NodeState()
        self._state_msg.nodename = rospy.get_name()
        self._state_msg.state    = NodeState.STARTING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = ""

        self._timer = Timer(1.0, self.heartbeat_timer_cb)
        self._timer.start()

    def publish_state(self):
        self._state_pub.publish(self._state_msg)

    def set_running(self):
        self._state_msg.state    = NodeState.RUNNING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = ""
        self.publish_state()

    def set_fatal(self, msg):
        self._state_msg.state    = NodeState.FATAL
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self.publish_state()

    def set_error(self, msg):
        self._state_msg.state    = NodeState.ERROR
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self.publish_state()

    def set_recovering(self, msg):
        self._state_msg.state    = NodeState.RECOVERING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self.publish_state()

    def heartbeat_timer_cb(self):
        if self._state_msg.state == NodeState.RUNNING:
            # we set the time automatically only when running!
            self._state_msg.time = rospy.Time.now()
        self.publish_state()  

        self._timer = Timer(1.0, self.heartbeat_timer_cb)
        self._timer.start()
