import os
import rospkg
import rospy
import signal
import subprocess

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsView, QWidget

class turtlebot_slam(Plugin):

    def __init__(self, context):
        super(turtlebot_slam, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('turtlebot_slam')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_turtlebot_slam'), 'resource', 'turtlebot_slam.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('turtlebot_slamUi')

        self._widget.loadfile.setIcon(QIcon.fromTheme('document-open'))
        self._widget.savefile.setIcon(QIcon.fromTheme('document-save'))

        self._widget.start_tbot.clicked[bool].connect(self._handle_start_tbot_clicked)
        self._widget.slam_algorithm.currentIndexChanged[str].connect(self._handle_slam_algorithm_changed)
        print type(self._widget.slam_algorithm)
        self._widget.wanderer.clicked[bool].connect(self._handle_wanderer_clicked)
        self._widget.keyboard_teleop.clicked[bool].connect(self._handle_keyboard_teleop_clicked)
        self._widget.loadfile.clicked[bool].connect(self._handle_loadfile_clicked)
        self._widget.savefile.clicked[bool].connect(self._handle_savefile_clicked)

        self._widget.start_tbot.setEnabled(True)
        self._widget.slam_algorithm.setEnabled(False)
        self._widget.wanderer.setEnabled(False)
        self._widget.keyboard_teleop.setEnabled(False)
        self._widget.loadfile.setEnabled(False)
        self._widget.savefile.setEnabled(False)

        self._widget.status_text.setReadOnly(True)

        #Booleans
        self.tbot_launched = False
        self.update_status = True

        #Subprocesses
        self.tbot_launch = None

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.status_period = 1
        rospy.Timer( rospy.Duration(self.status_period), self._timer_callback )

        self.status_update_text = "Select a button"
        self._widget.status_text.setText(self.status_update_text)


    def _timer_callback(self, event):
        self._status_update()

    def _status_update(self):
        if self.update_status:
            #self._widget.status_text.setText(self.status_update_text)
            self.update_status = False
        if type(self.tbot_launch) is not type(None):
            tbot_launch_status = self.tbot_launch.poll()
            if tbot_launch_status is type(None):
                self.status_update_text = "Turtlebot Started"
                self.update_status = True
            elif tbot_launch_status is not type(None) and not self.tbot_launched:
                self.status_update_text = "Turtlebot Error"
                self.update_status = True
            else:
                self.status_update_text = "Turtlebot Shutdown"
                self.update_status = True

    def _handle_start_tbot_clicked(self, checked):
        print type(self._widget.status_text)
        if self.tbot_launched:
            #os.killpg(os.getpgid(self.tbot_launch.pid), signal.SIGTERM)
            self.tbot_launched = False
            self._widget.slam_algorithm.setEnabled(False)
            self._widget.wanderer.setEnabled(False)
            self._widget.keyboard_teleop.setEnabled(False)
            self._widget.loadfile.setEnabled(False)
            self._widget.start_tbot.setText("Start Turtlebot")
            self._widget.status_text.setText("Turtlebot Stopped")
        else:
            self.ip_of_turtlebot = self._widget.ip_of_turtlebot.toPlainText()
            self.tbot_launch = subprocess.Popen("exec rosrun rqt_turtlebot_slam start_turtlebot "+self.ip_of_turtlebot, stdin=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
            #self.tbot_launch = subprocess.Popen("exec roslaunch rqt_turtlebot_slam turtlebot.launch", stdin=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
            self.tbot_launched = True
            self._widget.slam_algorithm.setEnabled(True)
            self._widget.wanderer.setEnabled(True)
            self._widget.keyboard_teleop.setEnabled(True)
            self._widget.loadfile.setEnabled(True)
            self._widget.start_tbot.setText("Stop Turtlebot")
            self._widget.status_text.setText("Turtlebot Started")
        print "Start Tbot Clicked"

    def _handle_slam_algorithm_changed(self, package=None):
        #self.slam_algorithm_launch = subprocess.Popen("exec rosrun rqt_turtlebot_slam slam_algorithm", stdin=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
        print "Slam Algorithm Changed:" + package

    def _handle_wanderer_clicked(self, checked):
        self.wanderer_launch = subprocess.Popen("exec roslaunch rqt_turtlebot_slam wanderer.launch", stdin=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
        print "RTabMap Clicked"

    def _handle_keyboard_teleop_clicked(self, checked):
        #self.keyboard_teleop_launch = subprocess.Popen("exec roslaunch rqt_turtlebot_slam orb_slam.launch", stdin=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
        print "Orb_Slam Clicked"

    def _handle_loadfile_clicked(self, checked):
        filename = QFileDialog.getOpenFileName(self._widget, self._widget.tr('Load from File'), '.', self._widget.tr('PGM files {.pgm} (*.pgm)'))
        self._widget.savefile.setEnabled(True)
        print "LoadFile Clicked"

    def _handle_savefile_clicked(self, checked):
        self._widget.savefile.setEnabled(False)
        print "SaveFile Clicked"

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

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
