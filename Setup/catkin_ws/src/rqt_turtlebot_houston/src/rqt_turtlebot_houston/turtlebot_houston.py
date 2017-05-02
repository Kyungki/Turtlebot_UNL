#!/usr/bin/env python

import roslib
import rospkg
import rospy
import rviz

import sys
import os

import cv2
import numpy as np

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt32
from std_srvs.srv import Empty

from apriltags.msg import AprilTagDetections 

from subprocess import Popen, PIPE

class rospy_thread(QThread):
    def __init__(self):
        QThread.__init__(self)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

class map_save_thread(QThread):
    def __init__(self, map_file="~/.ros/rtabmap", status_label=None, call_function=None, out_file="~/.ros/rtabmap.db"):
        QThread.__init__(self)
        self.update = None
        self.map_file = map_file
        self.out_file = out_file
        self.status_label = status_label
        self.call_function = call_function
        self.master_ip = Popen(["change_master"], stdout=PIPE).stdout.read().strip('\n').strip('Current Master: ')

    def run(self):
        cmd = ["dabit-launcher","sync_rtabmap",'"{}"'.format(self.out_file)]
        print cmd
        shell = Popen(cmd, stdout=PIPE, stderr=PIPE, stdin=PIPE)
        while True:
            output = shell.stdout.readline()
            print output
            self.status_label.setText(str(output))
            if output == '' and shell.poll() is not None:
                break
        self.call_function(True)

class map_load_thread(QThread):
    def __init__(self, filename, return_function):
        QThread.__init__(self)
        self.filename = filename
        self.return_function = return_function

    def run(self):
        cmd = ["dabit-launcher","xml_to_pgm",'"{}"'.format(self.filename)]
        shell = Popen(cmd, stdout=PIPE, stderr=PIPE, stdin=PIPE)
        while True:
            output = shell.stdout.readline()
            print output
            if output == '' and shell.poll() is not None:
                break
        image = "{}.pgm".format(self.filename[:-4])
        self.return_function(image)

class CvToQImage(QImage):
    def __init__(self, img, mapping="RGB"):
        height, width, channel = img.shape
        bytesPerLine = 3 * width
        self.mapping = mapping
        #self.qImg = QImage(img.data, width, height, bytesPerLine, QImage.Format_RGB888)
        super(CvToQImage, self).__init__(img, width, height, QImage.Format_RGB888)

class ImageView(QLabel):
    def __init__(self, parent=None):
        super(ImageView, self).__init__(parent)
        self._image = None

    def updateFrame(self, img, pixmap=False, height=180, width=135):
        self._image = img
        if self._image is not None:
            if pixmap:
                _image_pixmap = QPixmap(self._image)
            else:
                _image_pixmap = QPixmap.fromImage(self._image)
            _image_resized = _image_pixmap.scaled(height,width,Qt.KeepAspectRatio)
            self.setPixmap(_image_resized)

class teleop(QThread):
    def __init__(self):
        QThread.__init__(self)

	self.moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
                '16777235':(1,0), #up arrow
                '16777234':(0,1), #left arrow
                '16777236':(0,-1), #right arrow
                '16777237':(-1,0) #down arrow
		}

	self.speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		}

        self.speed = .15
        self.turn = .5

	self.x = 0
	self.th = 0
	self.status = 0
	self.count = 0
	self.acc = 0.01
        self.acc_turn = 0.05
	self.target_speed = 0
	self.target_turn = 0
	self.control_speed = 0
	self.control_turn = 0

        self.key = None
        self.moving = True

	self.pub_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=3)

    def stop(self):
	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	self.pub_vel.publish(twist)

    def destruct(self):
        self.stop()

    def pause(self):
        self.stop()
        self.moving = False

    def resume(self):
        self.moving = True

    def run(self):
        while not rospy.is_shutdown():
            if self.moving:
                key = self.key
                if key in self.moveBindings.keys():
                    self.x = self.moveBindings[key][0]
                    self.th = self.moveBindings[key][1]
                    self.count = 0
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    self.count = 0
                elif key == ' ' or key == 'k' :
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0
                    self.control_turn = 0
                else:
                    self.count = self.count + 1
                    if self.count > 8:
                        self.x = 0
                        self.th = 0
                    if (key == '\x03'):
                        self.close()

                self.target_speed = self.speed * self.x
                self.target_turn = self.turn * self.th

                if self.target_speed > self.control_speed:
                    self.control_speed = min(self.target_speed, self.control_speed + self.acc)
                elif self.target_speed < self.control_speed:
                    self.control_speed = max(self.target_speed, self.control_speed - self.acc)
                else:
                    self.control_speed = self.target_speed

                if self.target_turn > self.control_turn:
                    self.control_turn = min(self.target_turn, self.control_turn + self.acc_turn)
                elif self.target_turn < self.control_turn:
                    self.control_turn = max(self.target_turn, self.control_turn - self.acc_turn)
                else:
                    self.control_turn = self.target_turn

                if self.control_speed != 0 or self.control_turn != 0:
                    twist = Twist()
                    twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
                    self.pub_vel.publish(twist)
            rospy.sleep(rospy.Duration(0.025))

class PgmWidget(QWidget):

    def __init__(self, filename):
        QWidget.__init__(self)

        self.setWindowTitle("XML_TO_PGM")

        layout = QVBoxLayout()

        self.imageLabel = QLabel()
        self.imageLabel.setPixmap(QPixmap(filename))

        layout.addWidget(self.imageLabel)
        
        self.setLayout(layout)

class ImageWidget(QWidget):

    def __init__(self, image_topic, compressed=False):
        print "ImageWidget"
        QWidget.__init__(self)

        self.setWindowTitle("OpenCV Example")

        self.bridge = CvBridge()

        layout = QVBoxLayout()

        self.imageLabel = ImageView()
        layout.addWidget(self.imageLabel)

        self.setLayout(layout)

        if compressed:
            self.img_sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback, True)
        else:
            self.img_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

    def image_callback(self, img_msg, compressed=False):
        try:
            if compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "rgb8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.imageLabel.updateFrame(CvToQImage(cv_image),height=640,width=480)
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))

def RvizWidget(QWidget):

    def __init__(self,rviz_file="pcl.rviz"):
        print "RvizWidget"
        QWidget.__init__(self)

        self.setWindowTitle("PCL Example")

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        print "here1"

        layout = QVBoxLayout()

        file_rviz_config = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', rviz_file)
        file_icon_window = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'turtlebot_logo.png')

        self.setWindowIcon(QIcon(file_icon_window))
        print "here2"

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, file_rviz_config)

        self.frame.load(config)

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)
        print "here3"

        self.manager = self.frame.getManager()
        
        layout.addWidget(self.frame)

        self.setLayout(layout)
        print "here4"

    def test(self):
        print "test"


class turtlebot_houston(QWidget):

    def __init__(self):
        QWidget.__init__(self)

        rospy.init_node('turtlebot_houston_gui', anonymous=False)
        
        self.bridge = CvBridge()

        self.setObjectName('turtlebot_houston')
        rp = rospkg.RosPack()

        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        
        file_rviz_config = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'turtlebot_houston.rviz')
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
        
        self.map_status_label_action = QLabel("Using Map: ")
        self.map_status_label = QLabel("No File")
        h1_layout.addWidget(self.map_status_label_action)
        h1_layout.addWidget(self.map_status_label)
        
        h1_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))

        self.status_label = QLabel()
        self.status_label.setStyleSheet("QLabel { color : red; }");
        h1_layout.addWidget(self.status_label)

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

        self.button_mapping_resumepause = QPushButton("Pause Mapping")
        self.button_mapping_resumepause.clicked.connect(self.mappingPauseResume)
        self.button_mapping_resumepause.setFixedWidth(150)
        h2_layout.addWidget(self.button_mapping_resumepause)

        h2_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))

        self.button_turtlebot_startstop = QPushButton("Start Turtlebot")
        self.button_turtlebot_startstop.clicked.connect(self.turtlebotStartStop)
        self.button_turtlebot_startstop.setEnabled(False)
        self.button_turtlebot_startstop.setFixedWidth(150)
        h2_layout.addWidget(self.button_turtlebot_startstop)
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton("Top View")
        top_button.clicked.connect(self.onTopButtonClick)
        h_layout.addWidget(top_button)
        
        side_button = QPushButton("Side View")
        side_button.clicked.connect(self.onSideButtonClick)
        h_layout.addWidget(side_button)

        FPS_button = QPushButton("First Person View")
        FPS_button.clicked.connect(self.onFPSButtonClick)
        h_layout.addWidget(FPS_button)

        Free_button = QPushButton("Free View")
        Free_button.clicked.connect(self.onFreeButtonClick)
        h_layout.addWidget(Free_button)

        h3_layout = QHBoxLayout()

        #image_view
        self.imageLabel = ImageView("BGR")
        self.imageLabel2 = ImageView("BGR")

        teleop_label = ("Move your Turtlebot:\n"
                    "\tu \t i \t o\n"
                    "\tj \t k \t l\n"
                    "\tm \t , \t .\n\n"
                    "Space key, k: force stop\n"
                    "\tq/z: increase/decrease linear speed\n"
                    "\tw/x:increase\decrease linear speed\n"
                    "\te/c: increase/decrease angular speed")

        h3_layout.addWidget(self.imageLabel)
        h3_layout.addWidget(self.imageLabel2)
        h3_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))
        h3_layout.addWidget(QLabel(teleop_label))
        
        h4_layout = QHBoxLayout()

        self.detectionLabel = QLabel("No Detections")
        self.dustLabel = QLabel("No Dust Sensor")
        self.arduinoMillis = QLabel("0")

        h4_layout.addWidget(QLabel("Detected: "))
        h4_layout.addWidget(self.detectionLabel)
        h4_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))
        h4_layout.addWidget(QLabel("Arduino Millis(): "))
        h4_layout.addWidget(self.arduinoMillis)
        h4_layout.addWidget(QLabel("Dust Level: "))
        h4_layout.addWidget(self.dustLabel)

        h5_layout = QVBoxLayout()

        #self.opencv_list = ["OpenCV Test", "Facial Recognition", "Object Recognition"]
        self.opencv_list = ["OpenCV Test"]

        self.combobox_opencv = QComboBox()
        self.combobox_opencv.activated.connect(self.comboCV)
        self.combobox_opencv.addItems(self.opencv_list)
        self.combobox_opencv.setFixedWidth(120)
        
        #self.segmentation_list = ["PCL Test", "Planar Segmentation", "Cylinder Segmentation"]
        self.segmentation_list = ["PCL Test"]

        self.combobox_segmentation = QComboBox()
        self.combobox_segmentation.activated.connect(self.comboPCL)
        self.combobox_segmentation.addItems(self.segmentation_list)
        self.combobox_segmentation.setFixedWidth(120)

        self.button_wanderer_startstop = QPushButton("Start Wanderer")
        self.button_wanderer_startstop.clicked.connect(self.wandererStartStop)
        self.button_wanderer_startstop.setFixedWidth(120)

        self.button_bagging = QPushButton("Start Logging")
        self.button_bagging.clicked.connect(self.baggingStart)
        self.button_bagging.setEnabled(False)
        self.button_bagging.setFixedWidth(120)
        
        h5_layout.addWidget(self.combobox_opencv)
        h5_layout.addWidget(self.combobox_segmentation)
        h5_layout.addWidget(self.button_wanderer_startstop)
        h5_layout.addWidget(self.button_bagging)
        h5_layout.addItem(QSpacerItem(1,1,QSizePolicy.Minimum, QSizePolicy.Expanding))

        h3_layout.addLayout(h5_layout)

        layout.addLayout(h1_layout)
        layout.addLayout(h2_layout)
        layout.addWidget(self.frame)
        layout.addLayout(h_layout)
        layout.addLayout(h3_layout)
        layout.addLayout(h4_layout)
        
        self.setLayout(layout)

        self.tbot_launched = False
        self.bagging = False
        self.wanderer_launched = False
        self.mapping_paused = False

        self.rthread = rospy_thread()
        self.rthread.start()

        self.keyboard_teleop =  teleop()
        self.keyboard_teleop.start()
        self.key = None

        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback, (0, True), queue_size=1)
        self.img2_sub = rospy.Subscriber("/fiducial_markers/detections_image/compressed", CompressedImage, self.image_callback, (1, True), queue_size=1)

        self.detections_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.detections_callback, queue_size=1)

        self.millis_sub = rospy.Subscriber("/arduino/millis", UInt32, self.millis_callback, queue_size=1)

        self.master_ip = Popen(["change_master"], stdout=PIPE).stdout.read().strip('\n').strip('Current Master: ')


    def destruct(self):
        self.teleop.destruct()
	self.frame = None
	for p in self.props:
	    p.getParent().takeChild(p)
	self.props = []

    def millis_callback(self, msg):
        self.arduinoMillis.setText(str(msg.data))

    def image_callback(self, img_msg, args):
        img_sub = args[0]
        compressed = args[1]
        try:
            if compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "rgb8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

            if img_sub == 0:
                self.imageLabel.updateFrame(CvToQImage(cv_image))
            else:
                self.imageLabel2.updateFrame(CvToQImage(cv_image))
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def detections_callback(self, msg):
        if msg.detections:
            ids = []
            for detection in msg.detections:
                ids.append(detection.id)
            self.detectionLabel.setText(', '.join(map(str,ids)))
        else:
            self.detectionLabel.setText("No Detections")

    def eventFilter(self, source, event):
        if (event.type() == QEvent.KeyPress or event.type() == QEvent.ShortcutOverride):
            self.key = event.text()
            if self.key == "":
                self.key = event.key()
            if self.key == chr(27):
                self.close()
            self.keyboard_teleop.key = self.key
            return True
        if (event.type() == QEvent.KeyRelease):
            self.key = None
            self.keyboard_teleop.key = None
            return True
        return QWidget.eventFilter(self, source, event)

    def onTopButtonClick(self):
        self.switchToView("Top View");
        
    def onSideButtonClick(self):
        self.switchToView("Side View");

    def onFPSButtonClick(self):
        self.switchToView("FPS View");

    def onFreeButtonClick(self):
        self.switchToView("Free View");

    def turtlebotStartStop(self):
        if not self.tbot_launched:
            print "Turtlebot Start"
            self.tbot_launched = True
            self.button_turtlebot_startstop.setText("Stop Turtlebot")
            self.status_turtlebot.setText("Starting")
        else:
            print "Turtlebot Stop"
            self.tbot_launched = False
            self.button_turtlebot_startstop.setText("Start Turtlebot")
            self.status_turtlebot.setText("Stopping")

    def mappingPauseResume(self,task=0):
        if task==1:
            self.mapping_paused = False
        elif task==2:
            self.mapping_paused = True
        if not self.mapping_paused:
            print "Pausing Mapping"
            rospy.ServiceProxy('/rtabmap/pause',Empty).call()
            self.button_mapping_resumepause.setText("Resume Mapping")
            self.status_label.setText("Mapping Paused, Teleop Paused")
            self.keyboard_teleop.moving = False
            self.mapping_paused = True
        else:
            print "Resuming Mapping"
            rospy.ServiceProxy('/rtabmap/resume',Empty).call()
            self.button_mapping_resumepause.setText("Pause Mapping")
            self.status_label.setText("")
            self.keyboard_teleop.moving = True
            self.mapping_paused = False

    
    def saveFile(self):
        print "Save File"
        #self.mappingPauseResume(1)
        #self.button_mapping_resumepause.setEnabled(False)
        #self.map_status_label_action.setText("Saving Map: ")
        #self.save_thread = map_save_thread("~/.ros/rtabmap.db", self.map_status_label, self.fileSaved)
        #self.save_thread.start()
    
    def fileSaved(self, success):
        self.button_mapping_resumepause.setEnabled(True)
        self.mappingPauseResume(2)
        if success:
            print "File Saved! Starting Viewer"
        else:
            print "File Save Failed!"

    def loadFile(self):
        filename = QFileDialog.getOpenFileName(self, 'Open File', '/home', filter="*.xml")
        print "File Selected: {}".format(filename[0])
        #self.load_thread = map_load_thread(filename[0], self.fileLoaded)
        #self.load_thread.start()

    def fileLoaded(self,filename):
        print filename
        if filename:
            self.image_pgm = PgmWidget(str(filename))
            self.image_pgm.resize(400,400)
            self.image_pgm.show()

    def comboCV(self, package=None):
        # To-Do: Requires a new widget to show image in separate window
        pkg = self.opencv_list[package]
        print "CV: {}".format(pkg)
        if pkg == "OpenCV Test":
            self.image_opencv_example = ImageWidget("/roscpp_opencv/output_video", False)
            self.image_opencv_example.resize(400,400)
            self.image_opencv_example.show()

    def comboPCL(self, package=None):
        pkg = self.segmentation_list[package]
        print "Segment: {}".format(pkg)
#        if pkg == "PCL Test":
#            self.rviz_pcl_example = RvizWidget("pcl.rviz")
#            self.rviz_pcl_example.test()
            #self.rviz_pcl_example.resize(400,400)
#            self.rviz_pcl_example.show()

    def wandererStartStop(self):
        print ["ssh","turtlebot@{}".format(self.master_ip),"dabit-launcher","wanderer"]
        shell = Popen(["ssh","turtlebot@{}".format(self.master_ip),"dabit-launcher","wanderer"], stdout=PIPE, stderr=PIPE, stdin=PIPE)
        print shell.stdout.read()
        if not self.wanderer_launched:
            print "Wanderer Start"
            self.wanderer_launched = True
            self.button_wanderer_startstop.setText("Stop Wanderer")
        else:
            print "Wanderer Stop"
            self.wanderer_launched = False
            self.button_wanderer_startstop.setText("Start Wanderer")

    def baggingStart(self):
        print "Bagging Start"

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
