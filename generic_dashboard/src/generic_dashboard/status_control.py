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

from os import path
from generic_control import GenericControl

class StatusControl(GenericControl):
  def __init__(self, parent, icons_path, base_name, toggleable, states, default_state):
    GenericControl.__init__(self, parent)
    self.SetSize(wx.Size(32, 32))

    self._states = states
    self._bitmaps = {}
    for state, filekey in states.iteritems():
        bitmap = wx.Bitmap(path.join(icons_path, "%s-%s.png"%(base_name, filekey)), wx.BITMAP_TYPE_PNG)
        self._bitmaps[(state, False)] = bitmap
        if toggleable:
            bitmap = wx.Bitmap(path.join(icons_path, "%s-%s-toggled.png"%(base_name, filekey)), wx.BITMAP_TYPE_PNG)
            self._bitmaps[(state, True)] = bitmap

    self._color = default_state
    
    self.Bind(wx.EVT_PAINT, self.on_paint)
    self.Bind(wx.EVT_LEFT_UP, self.on_left_up)
    self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
    self.Bind(wx.EVT_LEAVE_WINDOW, self.on_leave_window)
    self.Bind(wx.EVT_ENTER_WINDOW, self.on_enter_window)
    
    self._toggled = False
    self._left_down = False
    
  def toggle(self, tog):
    if (self._toggled == tog):
        return False
    
    self._toggled = tog
    self.Refresh()
    
    return True

  def on_left_down(self, evt):
    self.toggle(True)
    self._left_down = True
    self.Refresh()

  def on_left_up(self, evt):
    self.toggle(False)
    self._left_down = False
    x = evt.GetX()
    y = evt.GetY()
    if (x >= 0 and y >= 0 and x < self.GetSize().GetWidth() and y < self.GetSize().GetHeight()):
      event = wx.CommandEvent(wx.EVT_BUTTON._getEvtType(), self.GetId())
      wx.PostEvent(self, event)
      
    self.Refresh()

  def on_leave_window(self, evt):
    self.toggle(False)
    self.Refresh()
    
  def on_enter_window(self, evt):
    if (self._left_down):
      self.toggle(True)
      
    self.Refresh()

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()

    if self._color is None:
        return
    
    size = self.GetSize()
    
    bitmap = self._bitmaps[ (self._color, self._toggled) ]
    
    dc.DrawBitmap(bitmap, (size.GetWidth() - bitmap.GetWidth()) / 2.0, (size.GetHeight() - bitmap.GetHeight()) / 2.0, True)

  def set_status(self, status):
    if self._color == status:
        return False
        
    self._color = status
    self.update()
    
    return True
    
  def update(self):
    self.Refresh()
