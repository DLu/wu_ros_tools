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
import wx
from status_control import *
from os import path
import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
from generic_control import GenericControl

class DiagnosticsFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(400, 600))
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    self._diagnostics_panel = RobotMonitorPanel(self)

  def on_close(self, evt):
    self.Hide()

class DiagnosticsControl(StatusControl):
    def __init__(self, parent):
        StatusControl.__init__(self, parent, path.join(roslib.packages.get_pkg_dir('generic_dashboard'), "icons/"), 
                                    'diag', True, {'stale': 'grey', 'ok': 'green', 'warn': 'yellow', 'error': 'red'}, 'stale')
        self.SetToolTip(wx.ToolTip("Diagnostics"))

        self._diagnostics_frame = DiagnosticsFrame(parent, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)

    def on_diagnostics_clicked(self, evt):
        self._diagnostics_frame.Show()
        self._diagnostics_frame.Raise()

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

      self.Refresh()
