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


class turtlebot_houston( QWidget ):

    def __init__(self):
        QWidget.__init__(self)

        self.setObjectName('turtlebot_houston')
        rp = rospkg.RosPack()

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        
        file_rviz_config = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'turtlebot_houston.rviz')
        file_icon_window = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'turtlebot_logo.png')
        file_icon_save = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'document-save.png')
        file_icon_open = os.path.join(rp.get_path('rqt_turtlebot_houston'), 'resource', 'document-open.png')

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, file_rviz_config )
        self.frame.load( config )

        self.setWindowTitle( "Turtlebot GUI" )
        self.setWindowIcon(QIcon(file_icon_window))

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        self.manager = self.frame.getManager()

        layout = QVBoxLayout()

        h1_layout = QHBoxLayout()
        h2_layout = QHBoxLayout()
        
        map_status_label = QLabel("No File")
        h1_layout.addWidget(QLabel("Using Map: "))
        h1_layout.addWidget(map_status_label)

        h1_layout.addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.status_turtlebot = QLabel( "Unknown" )
#        self.status_turtlebot.setFixedWidth(100)
        h1_layout.addWidget(QLabel("Turtlebot Status: "))
        h1_layout.addWidget( self.status_turtlebot )

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

        self.button_start_turtlebot = QPushButton( "Start Turtlebot" )
        self.button_start_turtlebot.clicked.connect( self.turtlebotStart )
        self.button_start_turtlebot.setFixedWidth(150)
        h2_layout.addWidget( self.button_start_turtlebot )
        
        self.button_stop_turtlebot = QPushButton( "Stop Turtlebot" )
        self.button_stop_turtlebot.clicked.connect( self.turtlebotStop )
        self.button_stop_turtlebot.setFixedWidth(150)
        self.button_stop_turtlebot.setEnabled(False)
        h2_layout.addWidget( self.button_stop_turtlebot )
        

        layout.addLayout( h1_layout )
        layout.addLayout(h2_layout)

        layout.addWidget( self.frame )
        
#        thickness_slider = QSlider( Qt.Horizontal )
#        thickness_slider.setTracking( True )
#        thickness_slider.setMinimum( 1 )
#        thickness_slider.setMaximum( 1000 )
#        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
#        layout.addWidget( thickness_slider )
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

        self.tbot_launched = False


#    def onThicknessSliderChanged( self, new_value ):
#        if self.grid_display != None:
#            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );

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
        
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

if __name__ == '__main__':
    try:
        app = QApplication( sys.argv )

        myviz = turtlebot_houston()
        myviz.resize( 800, 600 )
        myviz.show()
        app.exec_()
    except Exception, e:
        print e
