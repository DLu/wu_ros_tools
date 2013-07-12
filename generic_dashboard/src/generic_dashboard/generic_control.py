import roslib; roslib.load_manifest('generic_dashboard')
import rospy
import wx

class GenericControl(wx.Window):
    def __init__(self, parent, id=wx.ID_ANY, trigger=None, callback=None):
        wx.Window.__init__(self, parent, id)
        self.trigger = trigger
        self.callback = callback

    def set_status(self, status):
        None

    def update(self):
        self.Refresh()

    def set_data(self, data):
        None

