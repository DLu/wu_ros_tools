
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('generic_dashboard')
import rospy
import diagnostic_msgs.msg
import wx
from status_control import *
from os import path
import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel

class DiagnosticsFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(400, 600))
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    self._diagnostics_panel = RobotMonitorPanel(self)

  def on_close(self, evt):
    self.Hide()

class DiagnosticsControl(StatusControl):
    def __init__(self, parent):
        StatusControl.__init__(self, parent, wx.ID_ANY, path.join(roslib.packages.get_pkg_dir('generic_dashboard'), "icons/"), 
                                    'diag', True, {'stale': 'grey', 'ok': 'green', 'warn': 'yellow', 'error': 'red'}, 'stale')
        self.SetToolTip(wx.ToolTip("Diagnostics"))

        self._diagnostics_frame = DiagnosticsFrame(parent, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
    
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
        
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

    def on_diagnostics_clicked(self, evt):
        self._diagnostics_frame.Show()
        self._diagnostics_frame.Raise()
    

    def dashboard_callback(self, msg):
      wx.CallAfter(self.new_dashboard_message, msg)
      
    def new_dashboard_message(self, msg):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()

            
      battery_status = {}
      laptop_battery_status = {}
      breaker_status = {}
      op_mode = None

      for status in msg.status:
          if status.name == "/Power System/Battery":
              for value in status.values:
                  battery_status[value.key]=value.value
          if status.name == "/Power System/Laptop Battery":
              for value in status.values:
                  laptop_battery_status[value.key]=value.value
          if status.name == "/Mode/Operating Mode":
              op_mode=status.message
          if status.name == "/Digital IO/Digital Outputs":
              #print "got digital IO"
              for value in status.values:
                  breaker_status[value.key]=value.value

      """
      if (battery_status):
        self._power_state_ctrl.set_power_state(battery_status)
      else:
        self._power_state_ctrl.set_stale()

      if (laptop_battery_status):
        self._power_state_ctrl_laptop.set_power_state(laptop_battery_status)
      else:
        self._power_state_ctrl_laptop.set_stale()
      
      if (op_mode):
        if (op_mode=='Full'):
          if (self._motors_button.set_ok()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Full"))
        elif(op_mode=='Safe'):
          if (self._motors_button.set_warn()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Safe"))
        elif(op_mode=='Passive'):
          if (self._motors_button.set_error()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Passive"))
      else:
          if (self._motors_button.set_stale()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Stale"))

      if (breaker_status):
          [ctrl.set_breaker_state(breaker_status) for ctrl in self._breaker_ctrls]
        """

    def update(self):
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level == -1 or level == 3):
        if (self.set_status('stale')):
            self.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self.set_status('error')):
            self.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self.set_status('warn')):
            self.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self.set_status('ok')):
            self.SetToolTip(wx.ToolTip("Diagnostics: OK"))
      """
      if (rospy.get_time() - self._last_dashboard_message_time > 5.0):
          self._motors_button.set_stale()
          self._power_state_ctrl.set_stale()
          self._power_state_ctrl_laptop.set_stale()
          [ctrl.reset() for ctrl in self._breaker_ctrls]
#          self._runstop_ctrl.set_stale()
#          self._wireless_runstop_ctrl.set_stale()
          ctrls = [self._motors_button, self._power_state_ctrl, self._power_state_ctrl_laptop]
          ctrls.extend(self._breaker_ctrls)
          for ctrl in ctrls:
              ctrl.SetToolTip(wx.ToolTip("No message received on dashboard_agg in the last 5 seconds"))
"""
      self.Refresh()
