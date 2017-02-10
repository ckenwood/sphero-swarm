RUNNING_ROS = False
if RUNNING_ROS:
    import rospy
    import rosnode
    import roslib.scriptutil as scriptutil
    import rostopic
import os
import rospkg
import socket
import functools
import pydot
import ast
import getpass
import itertools
import copy
import StringIO
import yaml
import time
import re
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QTabWidget, QListWidget, QListWidgetItem, QGridLayout, QFormLayout
import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtCore as QtCore

import sphero


import logging
import sphero_control_logging
gui_logger = logging.getLogger("gui_logger")

#command#
# rqt --standalone rqt_grounding_and_analysis

class SpheroControl(Plugin):

    def __init__(self, context):
        super(SpheroControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SpheroControlPlugin')

        if RUNNING_ROS:
            # Process standalone plugin command-line arguments
            from argparse import ArgumentParser
            parser = ArgumentParser()
            # Add argument(s) to the parser.
            parser.add_argument("-q", "--quiet", action="store_true",
                          dest="quiet",
                          help="Put plugin in silent mode")
            args, unknowns = parser.parse_known_args(context.argv())
            if not args.quiet:
                gui_logger.info('arguments: {0}'.format(args))
                gui_logger.info('unknowns: {0}'.format(unknowns))

        # Create QWidget
        self._widget = QtGui.QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('sphero_control'), 'resource', 'controlmainwidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('sphero_control')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if RUNNING_ROS:
            if context.serial_number() > 1:
                self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            # Add widget to the user interface
            context.add_widget(self._widget)

        self._context = context
        self.s = None #reserved for sphero


        ########### CUSTOM CODE ############
        #self._widget.resizeEvent = self.onResize

        # list widgets
        self.listWidget_commands = self._widget.findChild(QtGui.QListWidget, name="listWidget_commands")

        # connect callback for pushButton direction comands
        self._pushButton_backward = self._widget.findChild(QtGui.QPushButton, name="pushButton_backward")
        self._pushButton_forward = self._widget.findChild(QtGui.QPushButton, name="pushButton_forward")
        self._pushButton_left = self._widget.findChild(QtGui.QPushButton, name="pushButton_left")
        self._pushButton_right = self._widget.findChild(QtGui.QPushButton, name="pushButton_right")

        self._pushButton_backward.clicked.connect(functools.partial(self.on_pushButton_direction_clicked, 'backward'))
        self._pushButton_forward.clicked.connect(functools.partial(self.on_pushButton_direction_clicked, 'forward'))
        self._pushButton_left.clicked.connect(functools.partial(self.on_pushButton_direction_clicked, 'left'))
        self._pushButton_right.clicked.connect(functools.partial(self.on_pushButton_direction_clicked, 'right'))

        # connect callback for pushButton execute comands
        self._pushButton_execute = self._widget.findChild(QtGui.QPushButton, name="pushButton_execute")
        self._pushButton_execute.clicked.connect(self.on_pushButton_execute_clicked)

        # connect callback for pushButton clear all comands
        self._pushButton_clearAll = self._widget.findChild(QtGui.QPushButton, name="pushButton_clearAll")
        self._pushButton_clearAll.clicked.connect(self.on_pushButton_clear_all_clicked)

        # connect callback for pushButton remove selected comands
        self._pushButton_removeSelected = self._widget.findChild(QtGui.QPushButton, name="pushButton_removeSelected")
        self._pushButton_removeSelected.clicked.connect(self.on_pushButton_remove_selected_clicked)

        # connect callback for pushButton connect comands
        self._pushButton_connect = self._widget.findChild(QtGui.QPushButton, name="pushButton_connect")
        self._pushButton_connect.clicked.connect(self.on_pushButton_connect_clicked)

        self._lineEdit_name = self._widget.findChild(QtGui.QLineEdit, name="lineEdit_name")
        self._lineEdit_name.setPlaceholderText("Sphero-GGW")

    def on_pushButton_connect_clicked(self):
        gui_logger.info('Sphero name is: {0}'.format(self._lineEdit_name.text()))
        r = re.compile('Sphero-.{3}')
        if r.match(self._lineEdit_name.text()) is not None:
            self.connect_to_sphero(self._lineEdit_name.text())
        else:
            gui_logger.warning('Sphero name is incorrect')

    def on_pushButton_remove_selected_clicked(self):
        item = self.listWidget_commands.currentItem()
        self.listWidget_commands.takeItem(self.listWidget_commands.row(item))

    def on_pushButton_clear_all_clicked(self):
        self.listWidget_commands.clear()

    def on_pushButton_direction_clicked(self, mode):
        item = QtGui.QListWidgetItem(mode)
        self.listWidget_commands.insertItem(self.listWidget_commands.count(), item)
        gui_logger.info('Got {0} command'.format(mode))

    def on_pushButton_execute_clicked(self):
        gui_logger.info('----Execute Commands!----')
        for idx in range(self.listWidget_commands.count()):
            gui_logger.info("Execute {0}".format(self.listWidget_commands.item(idx).text()))
            self.set_velocity(self.listWidget_commands.item(idx).text())

    def connect_to_sphero(self, target_name):
        target_name = 'Sphero-GGW'
        self.target_dict = {'Sphero-GGW':'68:86:E7:08:95:5F'}

        self.s = sphero.SpheroNode(target_name, self.target_dict[target_name], 1)
        gui_logger.info('Connecting to Sphero name: {0} ...'.format(target_name))
        self.s.start()
        gui_logger.info('Sphero name: {0} connected!'.format(self.target_dict[target_name]))

    def set_velocity(self, command):
        vx, vy = 0,0
        if command == 'left':
            vy = 50
        elif command == 'right':
            vy = -50
        elif command == 'forward':
            vx = 50
        elif command == 'backward':
            vx = -50

        start = time.time()
        while time.time()-start < 1: # not rospy.is_shutdown():
            self.s.core_cmd_vel(vx, vy)

        self.s.core_cmd_vel(0, 0)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        if self.s:
            self.s.stop()


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

if __name__ == "__main__":
    app = QtGui.QApplication([])
    window = QWidget()
    layout = QtGui.QVBoxLayout(window)
    a = SpheroControl(layout)
    a._widget.show()
    sys.exit(app.exec_())