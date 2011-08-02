
----------------------------------------------------------------------------
--  nodemon.lua - Node monitoring state publisher
--
--  Created: Fri Apr 15 14:57:30 2011
--  License: BSD, cf. LICENSE file of roslua
--  Copyright  2011  Tim Niemueller [www.niemueller.de]
--             2011  SRI International
--             2011  Carnegie Mellon University
--             2011  Intel Labs Pittsburgh
--             2011  Columbia University in the City of New York
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the 3-clase BSD License.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE file in the root directory.

--- Node monitoring state publisher.
-- This module provides an interface to send heartbeat messages and specific
-- error information.

-- To use this class, simply instantiate it in your code. As soon as
-- set_running() is called, the state publisher starts
-- to send periodical heartbeat messages. To switch the state, call any of
-- set_fatal(), set_error(), or set_recovering() for the respective state.
-- Call set_running() again to return to the normal operation state. Only in
-- this state the timestamp of the message is updated automatically. If you
-- want to indicate aliveness, for example during recovering, call the
-- respective set methods by yourself.
-- This module provides the Time class. It uses the local clock or the
-- /clock topic depending whether simulation time has been initialized or
-- not.
-- @copyright Tim Niemueller, SRI International, Carnegie Mellon University,
-- Intel Labs Pittsburgh
-- @release Released under BSD license
module("nodemon", package.seeall)

require("roslua")
require("posix")

NodeStatePublisher = {}

--- Constructor.
-- Create new node state publisher
-- @param package_name the name of the package that contains the node
-- @param node_type the node type, this is the name of the
-- executable, only the base name is used.
function NodeStatePublisher:new(package_name, node_type)
   assert(package_name, "NodeStatePublisher: No package name passed")
   assert(node_type, "NodeStatePublisher: No node type passed")

   local o = {}
   setmetatable(o, self)
   self.__index = self

   o.state_pub = roslua.publisher("/nodemon/state", "nodemon_msgs/NodeState", true)
   o.state_msg = o.state_pub.msgspec:instantiate()
   o.state_msg.values.nodename = roslua.get_name()
   o.state_msg.values.package  = package_name
   o.state_msg.values.nodetype = posix.basename(node_type)
   o.state_msg.values.state    = o.state_pub.msgspec.constants.STARTING.value
   o.state_msg.values.time     = roslua.Time.now()
   o.state_msg.values.machine_message  = ""
   o.state_msg.values.human_message    = ""
   o:publish_state()

   o.timer = roslua.timer(1.0, function (event)
				  o:heartbeat_timer_cb(event)
			       end)

   roslua.add_finalizer(o)

   return o
end

--- Finalize instance.
function NodeStatePublisher:finalize()
   roslua.registry.unregister_timer(self.timer)
   self.state_msg.values.state = self.state_pub.msgspec.constants.STOPPING.value
   self.state_msg.values.time  = roslua.Time.now()
   self.state_msg.values.machine_message  = ""
   self.state_msg.values.human_message    = ""
   self:publish_state()
end

--- Publish current state.
function NodeStatePublisher:publish_state()
   self.state_pub:publish(self.state_msg)
end

--- Set the running state.
-- The state publisher will start to send periodical heartbeat messages
-- with updated time stamps.
function NodeStatePublisher:set_running()
   self.state_msg.values.state    = self.state_pub.msgspec.constants.RUNNING.value
   self.state_msg.values.time     = roslua.Time.now()
   self.state_msg.values.machine_message  = ""
   self.state_msg.values.human_message    = ""
   self:publish_state()
end

--- Set the fatal state.
-- The fatal state is meant for errors from which the node cannot
-- recover without completely restarting it. The message should
-- give a meaningful and concise description of the cause of the
-- error.
-- @param machine_message machine parseable message describing the error
-- @param human_format message format describing the cause of the fatal error
-- @param ... arguments as required when passing format to string.format().
function NodeStatePublisher:set_fatal(machine_message, human_format, ...)
   local message = ""
   if human_format then message = string.format(human_format, ...) end
   self.state_msg.values.state    = self.state_pub.msgspec.constants.FATAL.value
   self.state_msg.values.time     = roslua.Time.now()
   self.state_msg.values.machine_message  = machine_message
   self.state_msg.values.human_message    = message
   self:publish_state()
end

--- Set the non-fatal error state.
-- The error state is meant for errors from which the node can
-- recover at run-time. The node may require input from other
-- nodes to start recovering, or it can start recovery by itself
-- if possible and without risk of damaging the robot or harming
-- humans. Once recovery is started, call set_recovering().  The
-- message should give a meaningful and concise description of
-- the cause of the error.
-- @param machine_message machine parseable message describing the error
-- @param human_format message format describing the cause of the fatal error
-- @param ... arguments as required when passing format to string.format().
function NodeStatePublisher:set_error(machine_message, human_format, ...)
   local message = ""
   if human_format then message = string.format(human_format, ...) end
   self.state_msg.values.state    = self.state_pub.msgspec.constants.ERROR.value
   self.state_msg.values.time     = roslua.Time.now()
   self.state_msg.values.machine_message  = machine_message
   self.state_msg.values.human_message    = message
   self:publish_state()
end

--- Set recovering state.
-- The recovery state is meant to describe that the node is in
-- the process of returning to an operational state. During that
-- time it cannot process any new requests or commands. Once
-- recovery is finished call set_running() to indicate that the
-- node is fully operational again.
-- @param machine_message machine parseable message describing the recovery
-- procedure briefly, e.g. "move_arm_safe_pos".
-- @param human_format message format describing the recovery method or
-- procedure briefly, e.g. "moving arm to safe position".
-- @param ... arguments as required when passing format to string.format().
function NodeStatePublisher:set_recovering(machine_message, human_format, ...)
   local message = ""
   if human_format then message = string.format(human_format, ...) end
   self.state_msg.values.state    =
      self.state_pub.msgspec.constants.RECOVERING.value
   self.state_msg.values.time     = roslua.Time.now()
   self.state_msg.values.machine_message  = machine_message
   self.state_msg.values.human_message    = message
   self:publish_state()
end

--- Send warning message.
-- Warning messages are meant to indicate a serious problem to the developer
-- or user, that the node was able to work around or solve this time, but
-- that might also cause trouble. The warning message is modeled as a
-- transitional state, i.e. an automatic transition is made back to the
-- previous state after the warning has been processed.
-- @param machine_msg a warning message describing the problem
-- briefly in a machine parseable format
-- @param human_format a warning message describing the problem
-- briefly in a human readable format
function NodeStatePublisher:send_warning(machine_message, human_format, ...)
   local message = ""
   if human_format then message = string.format(human_format, ...) end

   local oldmsg = self.state_msg:clone()

   self.state_msg.values.state    =
      self.state_pub.msgspec.constants.WARNING.value
   self.state_msg.values.time     = roslua.Time.now()
   self.state_msg.values.machine_message  = machine_message
   self.state_msg.values.human_message    = message
   self:publish_state()

   self.state_msg:copy(oldmsg)
   self:publish_state()
end

--- Timer callback triggering status publishing.
-- This function automatically restarts the time for as long as
-- ros is not shutdown.
function NodeStatePublisher:heartbeat_timer_cb(event)
   if self.state_msg.values.state ==
      self.state_pub.msgspec.constants.RUNNING.value
   then
      -- we set the time automatically only when running!
      self.state_msg.values.time = event.current_real
      self:publish_state()
   end
end
