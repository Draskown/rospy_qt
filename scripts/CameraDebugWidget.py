import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Bool
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

        # CV converter
        self.cvBridge = CvBridge()       

        # Publishers for the ROS topics
        self.front_mode_pub = rospy.Publisher('front_camera_mode', Int8, queue_size=1)
        self.line_mode_pub = rospy.Publisher('line_camera_mode', Int8, queue_size=1)
        self.show_env_pub = rospy.Publisher('show_env', Bool, queue_size=1)
        self.show_persp_pub = rospy.Publisher('show_persp', Bool, queue_size=1)
        self.show_line_det_pub = rospy.Publisher('show_line_det', Bool, queue_size=1)

        # Subscribers for the ROS topics   
        front_cam_sub = rospy.Subscriber("decided_img", Image, self.cb_front, queue_size=1)
        line_cam_sub = rospy.Subscriber("image", Image, self.cb_line, queue_size=1)

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
        self.front_camera_combo.addItems(["Original", "Traffic Light", "Bar", "Signs"])
        self.front_camera_combo.currentIndexChanged.connect(self.send_front_mode)
        self.line_camera_combo = QComboBox()
        self.line_camera_combo.addItems(["Original", "Mask White", "Mask Yellow"])
        self.line_camera_combo.currentIndexChanged.connect(self.send_line_mode)

        # Create switches
        self.environment_switch = QCheckBox('Show Environment')
        self.environment_switch.stateChanged.connect(self.send_show_env)
        self.line_detection_switch = QCheckBox('Show Line Detection')
        self.line_detection_switch.stateChanged.connect(self.send_show_line_det)
        self.perspective_switch = QCheckBox('Enable Perspective')
        self.perspective_switch.stateChanged.connect(self.send_show_persp)

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

        HLayout_switch.addWidget(self.environment_switch)
        HLayout_switch.addWidget(self.line_detection_switch)
        HLayout_switch.addWidget(self.perspective_switch)

        HLayout_label.addWidget(self.cam_label, alignment=Qt.AlignLeft)
        HLayout_label.addWidget(self.cam_line_label, alignment=Qt.AlignRight)

        # Create main layout for widget
        main_layout = QVBoxLayout()
        main_layout.addLayout(HLayout_label)
        main_layout.addLayout(Grid_combo)
        main_layout.addLayout(HLayout_switch)

        self.setLayout(main_layout)

    def cb_line(self, data):
        cv_img = self.cvBridge.imgmsg_to_cv2(data,"rgb8")
        h, w, ch = cv_img.shape
        bpl = ch*w
        qimage = QImage(cv_img.data, w, h, bpl, QImage.Format_RGB888)
        p = QPixmap.fromImage(qimage.scaled(int(640*0.75), int(640*0.75)))
        self.cam_line_label.setPixmap(p)

    def cb_front(self, data):
        cv_img = self.cvBridge.imgmsg_to_cv2(data,"rgb8")
        h, w, ch = cv_img.shape
        bpl = ch*w
        qimage = QImage(cv_img.data, w, h, bpl, QImage.Format_RGB888)
        p = QPixmap.fromImage(qimage.scaled(int(640*0.75), int(640*0.75)))
        self.cam_label.setPixmap(p)

    def send_front_mode(self):
        self.front_mode_pub.publish(self.front_camera_combo.currentIndex()+1)

    def send_line_mode(self):
        self.line_mode_pub.publish(self.line_camera_combo.currentIndex()+1)

    def send_show_env(self):
        self.show_env_pub.publish(self.environment_switch.isChecked())

    def send_show_persp(self):
        self.show_persp_pub.publish(self.perspective_switch.isChecked())
        
    def send_show_line_det(self):
        self.show_line_det_pub.publish(self.line_detection_switch.isChecked())
    ###
