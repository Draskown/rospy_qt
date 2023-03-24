import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget,
                             QLabel,
                             QVBoxLayout,
                             QHBoxLayout,
                             QGridLayout,
                             QComboBox,
                             QCheckBox,
                             QSizePolicy,
                             QApplication)
from PyQt5.QtGui import QPixmap, QImage
from Common_defs import set_icon, key_pressed


class CameraDebugWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.init_UI()

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
        # Set an icon
        set_icon(self)

        # Set Window Name
        self.setWindowTitle("Camera Debug Widget")

        # Create image outputs
        self.cam_label = QLabel()
        self.cam_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.cam_line_label = QLabel()
        self.cam_line_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        # Create comboboxes
        self.front_camera_combo = QComboBox()
        self.line_camera_combo = QComboBox()

        # Create switches
        environment_switch = QCheckBox('Show Environment')
        line_detection_switch = QCheckBox('Show Line Detection')
        perspective_switch = QCheckBox('Enable Perspective')

        # Create layouts for comboboxes and switches
        Grid_combo = QGridLayout()
        HLayout_switch = QHBoxLayout()
        HLayout_label = QHBoxLayout()

        Grid_combo.addWidget(QLabel('Front Camera Mode:'), 0, 0,
                             alignment=Qt.AlignCenter)
        Grid_combo.addWidget(self.front_camera_combo, 1, 0)
        Grid_combo.addWidget(QLabel('Line Camera Mode:'), 0, 1,
                             alignment=Qt.AlignCenter)
        Grid_combo.addWidget(self.line_camera_combo, 1, 1)

        HLayout_switch.addWidget(environment_switch)
        HLayout_switch.addWidget(line_detection_switch)
        HLayout_switch.addWidget(perspective_switch)

        HLayout_label.addWidget(self.cam_label, alignment=Qt.AlignLeft)
        HLayout_label.addWidget(self.cam_line_label, alignment=Qt.AlignRight)

        # Create main layout for widget
        main_layout = QVBoxLayout()
        main_layout.addLayout(HLayout_label)
        main_layout.addLayout(Grid_combo)
        main_layout.addLayout(HLayout_switch)

        self.setLayout(main_layout)

        # CV converter
        self.cvBridge = CvBridge()       
        # A subscriber for a ROS topic
        cam_sub = rospy.Subscriber("/camera/image", Image, self.cb_cam, queue_size=1)    
        cam_line_sub = rospy.Subscriber("/camera_line/image", Image, self.cb_line, queue_size=1)    

    def cb_line(self, data):
        cv_img = self.cvBridge.imgmsg_to_cv2(data,"rgb8")
        h, w, ch = cv_img.shape
        bpl = ch*w
        qimage = QImage(cv_img.data, w, h, bpl, QImage.Format_RGB888)
        p = QPixmap.fromImage(qimage.scaled(int(640*0.75), int(640*0.75)))
        self.cam_line_label.setPixmap(p)

    def cb_cam(self, data):
        cv_img = self.cvBridge.imgmsg_to_cv2(data,"rgb8")
        h, w, ch = cv_img.shape
        bpl = ch*w
        qimage = QImage(cv_img.data, w, h, bpl, QImage.Format_RGB888)
        p = QPixmap.fromImage(qimage.scaled(int(640*0.75), int(640*0.75)))
        self.cam_label.setPixmap(p)
    ###
