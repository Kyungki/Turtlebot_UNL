#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospkg
import rospy
import rviz

import sys
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

from geometry_msgs.msg import Twist

class teleop(QThread):
    def __init__(self):
        QThread.__init__(self)

        rospy.init_node('turtlebot_houston_gui_teleop',anonymous=False)

        print "Teleop Initiated"

	self.moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
		}

	self.speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		}

        self.speed = .2
        self.turn = .9

	self.x = 0
	self.th = 0
	self.status = 0
	self.count = 0
	self.acc = 0.1
	self.target_speed = 0
	self.target_turn = 0
	self.control_speed = 0
	self.control_turn = 0

        self.key = None

	self.pub_vel = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    def destruct(self):
	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	self.pub_vel.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            if self.key is not None:
                if self.key in self.moveBindings.keys():
                    self.x = self.moveBindings[self.key][0]
                    self.th = self.moveBindings[self.key][1]
                    self.count = 0
                elif self.key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[self.key][0]
                    self.turn = self.turn * self.speedBindings[self.key][1]
                    self.count = 0
                elif self.key == ' ' or self.key == 'k' :
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0
                    self.control_turn = 0
                else:
                    self.count = self.count + 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if (self.key == '\x03'):
                        self.close()

#                self.target_speed = self.speed * self.x
#                self.target_turn = self.turn * self.th
#
#                if self.target_speed > self.control_speed:
#                    self.control_speed = min(self.target_speed, self.control_speed + 0.02)
#                elif self.target_speed < self.control_speed:
#                    self.control_speed = max(self.target_speed, self.control_speed - 0.02)
#                else:
#                    self.control_speed = self.target_speed
#
#                if self.target_turn > self.control_turn:
#                    self.control_turn = min(self.target_turn, self.control_turn + 0.1)
#                elif self.target_turn < self.control_turn:
#                    self.control_turn = max(self.target_turn, self.control_turn - 0.1)
#                else:
#                    self.control_turn = self.target_turn
#
#                twist = Twist()
#                twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
#                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
#                self.pub_vel.publish(twist)

class turtlebot_houston(QWidget):

    def __init__(self):
        QWidget.__init__(self)


        self.setObjectName('turtlebot_houston')
        rp = rospkg.RosPack()

        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        
        file_rviz_config = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'test.rviz')
        file_icon_window = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'turtlebot_logo.png')
        file_icon_save = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'document-save.png')
        file_icon_open = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'document-open.png')

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, file_rviz_config)
        self.frame.load(config)

        self.setWindowTitle("Turtlebot GUI")
        self.setWindowIcon(QIcon(file_icon_window))

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.installEventFilter(self)
        self.frame.setFocusPolicy(Qt.ClickFocus)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()
        self.manager.installEventFilter(self)

        layout = QVBoxLayout()

        h1_layout = QHBoxLayout()
        h2_layout = QHBoxLayout()
        
        map_status_label = QLabel("No File")
        h1_layout.addWidget(QLabel("Using Map: "))
        h1_layout.addWidget(map_status_label)

        h1_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.status_turtlebot = QLabel("Unknown")
        h1_layout.addWidget(QLabel("Turtlebot Status: "))
        h1_layout.addWidget(self.status_turtlebot)

        self.button_save_file = QPushButton("")
        self.button_save_file.setIcon(QIcon(file_icon_save))
        self.button_save_file.setFixedWidth(28)
        self.button_save_file.clicked.connect(self.saveFile)
        h2_layout.addWidget(self.button_save_file)

        self.button_load_file = QPushButton("")
        self.button_load_file.setIcon(QIcon(file_icon_open))
        self.button_load_file.setFixedWidth(28)
        self.button_load_file.clicked.connect(self.loadFile)
        h2_layout.addWidget(self.button_load_file)

        h2_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))

        self.button_start_turtlebot = QPushButton("Start Turtlebot")
        self.button_start_turtlebot.clicked.connect(self.turtlebotStart)
        self.button_start_turtlebot.setFixedWidth(150)
        h2_layout.addWidget(self.button_start_turtlebot)
        
        self.button_stop_turtlebot = QPushButton("Stop Turtlebot")
        self.button_stop_turtlebot.clicked.connect(self.turtlebotStop)
        self.button_stop_turtlebot.setFixedWidth(150)
        self.button_stop_turtlebot.setEnabled(False)
        h2_layout.addWidget(self.button_stop_turtlebot)
        
        layout.addLayout(h1_layout)
        layout.addLayout(h2_layout)

        layout.addWidget(self.frame)
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton("Top View")
        top_button.clicked.connect(self.onTopButtonClick)
        h_layout.addWidget(top_button)
        
        side_button = QPushButton("Side View")
        side_button.clicked.connect(self.onSideButtonClick)
        h_layout.addWidget(side_button)
        
        layout.addLayout(h_layout)
        
        self.setLayout(layout)

        self.tbot_launched = False

        self.keyboard_teleop =  teleop()
        self.keyboard_teleop.start()
        self.key = None

    def destruct( self ):
        self.teleop.destruct()
	self.frame = None
	for p in self.props:
	    p.getParent().takeChild(p)
	self.props = []

    def eventFilter(self, source, event):
        if (event.type() == QEvent.KeyPress or event.type() == QEvent.ShortcutOverride):
            self.key = event.text()
            print('key pressed: {}'.format(self.key))
            if self.key == Qt.Key_Escape:
                self.close()
            #self.keyboard_teleop.key = self.key
            return True
        if (event.type() == QEvent.KeyRelease):
            self.key = None
            #self.keyboard_teleop.key = None
            return True
        return QWidget.eventFilter(self, source, event)

    def onTopButtonClick(self):
        self.switchToView("Top View");
        
    def onSideButtonClick(self):
        self.switchToView("Side View");

    def turtlebotStart(self):
        print "Turtlebot Start"
        self.tbot_launched = True
        self.button_start_turtlebot.setEnabled(False)
        self.button_stop_turtlebot.setEnabled(True)
        self.status_turtlebot.setText("Starting")

    def turtlebotStop(self):
        print "Turtlebot Stop"
        self.tbot_launched = False
        self.button_start_turtlebot.setEnabled(True)
        self.button_stop_turtlebot.setEnabled(False)
        self.status_turtlebot.setText("Stopping")
    
    def saveFile(self):
        print "Save File"
    
    def loadFile(self):
        print "Load File"
        
    def switchToView(self, view_name):
        view_man = self.manager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print("Did not find view named %s." % view_name)

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)

        myviz = turtlebot_houston()
        myviz.resize(800, 600)
        myviz.show()
	app.aboutToQuit.connect(myviz.destruct)
        app.exec_()
        myviz.destruct()
    except Exception, e:
        print e
