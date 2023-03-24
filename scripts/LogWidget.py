import rospy, cv2
from std_msgs.msg import String
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget,
                             QLabel,
                             QVBoxLayout,
                             QApplication)
from PyQt5.QtGui import QPixmap, QImage
from Common_defs import set_icon, key_pressed


class LogWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.init_UI()

        self.log = ""

    # Events
    def keyPressEvent(self, e):
        key_pressed(self, e)

    # Clear focus if mouse pressed on an empty space
    def mousePressEvent(self, e):
        if QApplication.focusWidget() is not None:
            QApplication.focusWidget().clearFocus()
    ###


    # Methods
    def init_UI(self):
        # Set Window Name
        self.setWindowTitle("Log Widget")

        # Fix the size of the window
        self.setFixedSize(250, 400)

        # Create label for the log output
        self.log_label = QLabel()

        # Set an icon
        set_icon(self)

        # Create QVBoxLayout for layout
        layout = QVBoxLayout()

        # Add label to layout
        layout.addWidget(self.log_label, alignment=Qt.AlignTop)

        # Set layout for widget
        self.setLayout(layout)

        # CV converter
        self.cvBridge = CvBridge()       
        # A subscriber for a ROS topic
        log_sub = rospy.Subscriber("log_msg", String, self.put_log, queue_size=1)  
    
    def put_log(self, data):
        if data.data != "":
            self.log += data.data
            self.log_label.setText(self.log)
    ###