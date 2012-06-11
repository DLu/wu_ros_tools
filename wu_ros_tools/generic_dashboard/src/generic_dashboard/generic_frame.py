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

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import std_msgs.msg
import std_srvs.srv
import rospy
from roslib import rosenv
from diagnostics_control import DiagnosticsControl
import diagnostic_msgs.msg
import collections
from os import path
import threading

class GenericFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent=None, id=wx.ID_ANY, title='Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP, app = wx.App()):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("dashboard_cpp", anonymous=True)
        except AttributeError:
            pass

        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        
        self.SetTitle('%s (%s)'%(title, rosenv.get_master_uri()))
        self._app = app
        self._triggers = collections.defaultdict(list)

    def init(self, components):
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)

        self._components_list = []

        self.recursive_init(sizer, components)
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Breakers"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        self._config = wx.Config("dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()

        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)

    def recursive_init(self, parent, components):
        if type(components)==type({}):
            for category in components:
                static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, category), wx.HORIZONTAL)
                parent.Add(static_sizer, 0)
                subcomps = self.recursive_init(static_sizer, components[category])
        elif type(components)==type([]):
            for component in components:
                parent.Add(component, 0)
                self.add(component)

    def add(self, component):
        self._components_list.append(component)
        if type(component) == DiagnosticsControl:
            self._triggers['*'].append(component)
        elif component.trigger:
            self._triggers[ component.trigger ].append( component )
    

    def start(self):
        self.Show()
        self._app.MainLoop()
        
    def __del__(self):
        self._dashboard_agg_sub.unregister()
        
    def on_timer(self, evt):
        if (rospy.get_time() - self._last_dashboard_message_time > 5.0):
            for component in self._components_list:
                component.set_status('stale')

        for component in self._components_list:
            component.update()
        
        if (rospy.is_shutdown()):
           self.Close()

    def dashboard_callback(self, msg):
        wx.CallAfter(self.new_dashboard_message, msg)
      
    def new_dashboard_message(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()

        for status in msg.status:
            path = status.name.split(':')[0]
            if path in self._triggers:
                for component in self._triggers[path]:
                    if component.callback:
                        component.set_data( component.callback(status) )
        """
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
                  breaker_status[value.key]=value.value"""

    """
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
        
    """

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
      
