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
    """ Node state publisher.
    This class provides an interface to send heartbeat messages and specific
    error information.

    To use this class, simply instantiate it in your code. As soon as
    set_running() is called, the state publisher starts
    to send periodical heartbeat messages. To switch the state, call any of
    set_fatal(), set_error(), or set_recovering() for the respective state.
    Call set_running() again to return to the normal operation state. Only in
    this state the timestamp of the message is updated automatically. If you
    want to indicate aliveness, for example during recovering, call the
    respective set methods by yourself.

    @author Tim Niemueller
    """

    def __init__(self):
        """ Constructor. """
        self._state_pub = rospy.Publisher('/nodemon/state', NodeState);
        
        self._state_msg = NodeState()
        self._state_msg.nodename = rospy.get_name()
        self._state_msg.state    = NodeState.STARTING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = ""

        self._timer = Timer(1.0, self.heartbeat_timer_cb)
        self._timer.start()

    def __del__(self):
        """ Destructor. """
        if self._timer.is_alive():
            self._timer.cancel()

    def _publish_state(self):
        """Publish current state. """
        self._state_pub.publish(self._state_msg)

    def set_running(self):
        """ Set the running state.
        The state publisher will start to send periodical heartbeat messages
        with updated time stamps.
        """
        self._state_msg.state    = NodeState.RUNNING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = ""
        self._publish_state()

    def set_fatal(self, msg):
        """ Set the fatal state.
        The fatal state is meant for errors from which the node cannot
        recover without completely restarting it. The message should
        give a meaningful and concise description of the cause of the
        error.
        @param msg message describing the cause of the fatal error
        """
        self._state_msg.state    = NodeState.FATAL
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self._publish_state()

    def set_error(self, msg):
        """ Set the non-fatal error state.
        The error state is meant for errors from which the node can
        recover at run-time. The node may require input from other
        nodes to start recovering, or it can start recovery by itself
        if possible and without risk of damaging the robot or harming
        humans. Once recovery is started, call set_recovering().  The
        message should give a meaningful and concise description of
        the cause of the error.
        @param msg message describing the cause of the fatal error
        """
        self._state_msg.state    = NodeState.ERROR
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self._publish_state()

    def set_recovering(self, msg):
        """ Set recovering state.
        The recovery state is meant to describe that the node is in
        the process of returning to an operational state. During that
        time it cannot process any new requests or commands. Once
        recovery is finished call set_running() to indicate that the
        node is fully operational again.
        @param msg a message describing the recovery method or
        procedure briefly, e.g. "moving arm to safe position".
        """
        self._state_msg.state    = NodeState.RECOVERING
        self._state_msg.time     = rospy.Time.now()
        self._state_msg.message  = msg
        self._publish_state()

    def heartbeat_timer_cb(self):
        """ Timer callback triggering status publishing.
        This function automatically restarts the time for as long as
        ros is not shutdown.
        """
        if self._state_msg.state == NodeState.RUNNING:
            # we set the time automatically only when running!
            self._state_msg.time = rospy.Time.now()
        self._publish_state()  

        if not rospy.is_shutdown():
            self._timer = Timer(1.0, self.heartbeat_timer_cb)
            self._timer.start()
