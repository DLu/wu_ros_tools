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

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import diagnostic_msgs.msg
import std_msgs.msg
import std_srvs.srv
import rospy
from roslib import rosenv

from os import path
import threading

class GenericFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent=None, id=wx.ID_ANY, title='Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
        
        self.SetTitle('%s (%s)'%(title, rosenv.get_master_uri()))

    def init(self, components):        
        #TODO
        icons_path = path.join(roslib.packages.get_pkg_dir('turtlebot_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)

        self.recursive_init(sizer, components)
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Breakers"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        self._config = wx.Config("dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        """
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        """
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)
    
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
        
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

    def recursive_init(self, parent, components):
        if type(components)==type({}):
            for category in components:
                static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, category), wx.HORIZONTAL)
                parent.Add(static_sizer, 0)
                subcomps = self.recursive_init(static_sizer, components[category])
        elif type(components)==type([]):
            for component in components:
                parent.Add(component, 0)
        
    def __del__(self):
        self._dashboard_agg_sub.unregister()
        
    def on_timer(self, evt):
      """
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level == -1 or level == 3):
        if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))
        
      self.update_rosout()
      
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
              ctrl.SetToolTip(wx.ToolTip("No message received on dashboard_agg in the last 5 seconds"))"""
        
      if (rospy.is_shutdown()):
        self.Close()
        
      
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
    def on_motors_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_passive_mode, menu.Append(wx.ID_ANY, "Passive Mode"))
      menu.Bind(wx.EVT_MENU, self.on_safe_mode, menu.Append(wx.ID_ANY, "Safety Mode"))
      menu.Bind(wx.EVT_MENU, self.on_full_mode, menu.Append(wx.ID_ANY, "Full Mode"))
      self._motors_button.toggle(True)
      self.PopupMenu(menu)
      self._motors_button.toggle(False)
      
    def on_passive_mode(self, evt):
      passive = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",turtlebot_node.srv.SetTurtlebotMode )
      try:
        passive(turtlebot_node.msg.TurtlebotSensorState.OI_MODE_PASSIVE)
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the turtlebot in passive mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def on_safe_mode(self, evt):
      safe = rospy.ServiceProxy("/turtlebot_node/set_operation_mode",turtlebot_node.srv.SetTurtlebotMode)
      try:
        safe(turtlebot_node.msg.TurtlebotSensorState.OI_MODE_SAFE)
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the turtlebot in safe mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def on_full_mode(self, evt):
      full = rospy.ServiceProxy("/turtlebot_node/set_operation_mode", turtlebot_node.srv.SetTurtlebotMode)
      try:
        full(turtlebot_node.msg.TurtlebotSensorState.OI_MODE_FULL)
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the turtlebot in full mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      


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
      
          
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      
