import rospy, roslaunch
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from PyQt5.QtGui import (QPalette,
                         QColor,
                         QPixmap,
                         QFont,
                         QImage)
from PyQt5.QtWidgets import (QApplication,
                             QMainWindow,
                             QPushButton,
                             QLabel,
                             QWidget,
                             QHBoxLayout,
                             QVBoxLayout,
                             QMessageBox)
from PyQt5.QtCore import (QThread,
                          QObject,
                          pyqtSignal,
                          Qt)

from CameraDebugWidget import CameraDebugWidget
from LidarWidget import LidarWidget
from LocationInfoWidget import LocationInfoWidget
from ManualMovementWidget import ManualMovementWidget
from LogWidget import LogWidget
from Common_defs import set_icon, key_pressed


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.init_UI()

    def init_UI(self):       
        # Set window title
        self.setWindowTitle('Main Window')

        # Set an icon
        set_icon(self)

        # Create central widget and set it as the main widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Create additional windows as Widgets
        self.camera_debug_widget = CameraDebugWidget()
        self.lidar_widget = LidarWidget()
        self.location_info_widget = LocationInfoWidget()
        self.manual_movement_widget = ManualMovementWidget()
        self.log_widget = LogWidget()

        # Create layout for central widget
        main_layout = QHBoxLayout()

        # Set image
        self.image_label = QLabel()

        # Create Camera Debug button
        camera_debug_button = QPushButton('Camera Debug')

        # Create Lidar button
        lidar_button = QPushButton('Lidar')

        # Create Location Info button
        location_info_button = QPushButton('Location Info')

        # Create Manual Movement button
        self.manual_movement_button = QPushButton('Manual Movement')

        # Create Start Mission button
        self.start_mission_button = QPushButton('Start Mission')

        # Create Open log button
        self.open_log = QPushButton('Log')
        
        # Init the status bar thread
        self.worker = Worker()
        self.worker.message.connect(self.statusBar().showMessage)

        # Move it to a thread
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)

        # Connect the message from worker to the thread
        self.thread.started.connect(self.worker.show_message)
        self.thread.start()
        
        # Add Items to the first column in a HLayout
        column1 = QVBoxLayout()
        column1.addStretch()
        column1.addWidget(camera_debug_button)
        column1.addWidget(lidar_button)
        column1.addStretch()

        # Add Items to the second column in a HLayout
        column2 = QVBoxLayout()
        column2.addWidget(self.image_label, Qt.AlignTop)
        column2.addWidget(self.start_mission_button, 5, Qt.AlignHCenter)

        # Add Items to the third column in a HLayout
        column3 = QVBoxLayout()
        column3.addStretch()
        column3.addWidget(location_info_button, Qt.AlignTop)
        column3.addWidget(self.manual_movement_button, Qt.AlignTop)
        column3.addStretch()
        column3.addWidget(self.open_log, Qt.AlignBottom)

        # Add those columns into the HLayout
        main_layout.addLayout(column1)
        main_layout.addLayout(column2)
        main_layout.addLayout(column3)

        # Set layout for central widget
        central_widget.setLayout(main_layout)

        # Create connections for buttons
        camera_debug_button.clicked.connect(self.show_camera_debug_widget)
        lidar_button.clicked.connect(self.show_lidar_widget)
        location_info_button.clicked.connect(self.show_location_info_widget)
        self.manual_movement_button.clicked.connect(self.show_manual_movement_widget)
        self.open_log.clicked.connect(self.show_log_wdiget)
        
        self.start_mission_button.clicked.connect(self.start_mission)

        # An object of image conversion
        self.cvBridge = CvBridge()
        # ROS topic subscribers
        camera_sub = rospy.Subscriber('/camera/image', Image, self.cb_cam, queue_size=1)
        plan_sub = rospy.Subscriber('plan', Bool, self.cb_plan, queue_size=1)
        log_sub = rospy.Subscriber('log_msg', String, self.cb_log, queue_size=1)
        
		# Set a publisher for the robot's velocity
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    # Events
    def keyPressEvent(self, e):
        key_pressed(self, e)

    # Clear focus if mouse pressed on an empty space
    def mousePressEvent(self, e):
        if QApplication.focusWidget() is not None:
            QApplication.focusWidget().clearFocus()

    def closeEvent(self, e):
        reply = QMessageBox.question(self, 'Window Close', 'Are you sure you want to close the app?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            e.accept()
            for win in QApplication.topLevelWidgets():
                win.close()
        else: e.ignore()
    ###

    # Methods
    def cb_plan(self, data):
        self.start = data.data

    def cb_log(self, data):
        self.worker.msg.append(data.data)

    def start_mission(self):
        self.manual_movement_button.setDisabled(True)
        self.location_info_widget.set_position_btn.setDisabled(True)
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/ros_workspace/src/rospy_qt/launch/rospy_qt.launch"])
        launch.start()
        
    def show_camera_debug_widget(self):
        self.camera_debug_widget.show()

    def show_lidar_widget(self):
        self.lidar_widget.show()

    def show_location_info_widget(self):
        self.location_info_widget.show()

    def show_manual_movement_widget(self):
        self.manual_movement_widget.show()

    def show_log_wdiget(self):
        self.log_widget.show()

    def cb_cam(self, img):
        cv_img = self.cvBridge.imgmsg_to_cv2(img,"rgb8")
        h, w, ch = cv_img.shape
        bpl = ch*w
        qimage = QImage(cv_img.data, w, h, bpl, QImage.Format_RGB888)
        p = QPixmap.fromImage(qimage.scaled(int(640*0.9), int(480*0.9)))
        self.image_label.setPixmap(p)
    ###


# Class for handling the displayed log msg
class Worker(QObject):
    message = pyqtSignal(str)
    msg = []

    def show_message(self):
        while True:
            if len(self.msg) > 0:
                self.message.emit(self.msg.pop(0))
                QThread.sleep(1)
            else:
                self.message.emit("")


def set_style(app):
    app.setStyle("Fusion")

    app.setFont(QFont("Gilroy", 11))
    dark_palette = QPalette()
    WHITE = QColor(255, 255, 255)
    BLACK = QColor(0, 0, 0)
    RED = QColor(255, 0, 0)
    PRIMARY = QColor(53, 53, 53)
    SECONDARY = QColor(25, 25, 25)
    LIGHT_PRIMARY = QColor(100, 100, 100)
    TERTIARY = QColor(42, 130, 218)
    dark_palette.setColor(QPalette.Window, PRIMARY)
    dark_palette.setColor(QPalette.WindowText, WHITE)
    dark_palette.setColor(QPalette.Base, SECONDARY)
    dark_palette.setColor(QPalette.AlternateBase, PRIMARY)
    dark_palette.setColor(QPalette.ToolTipBase, WHITE)
    dark_palette.setColor(QPalette.ToolTipText, WHITE)
    dark_palette.setColor(QPalette.Text, WHITE)
    dark_palette.setColor(QPalette.Button, LIGHT_PRIMARY)
    dark_palette.setColor(QPalette.ButtonText, WHITE)
    dark_palette.setColor(QPalette.BrightText, RED)
    dark_palette.setColor(QPalette.Link, TERTIARY)
    dark_palette.setColor(QPalette.Highlight, TERTIARY)
    dark_palette.setColor(QPalette.HighlightedText, BLACK)
    app.setPalette(dark_palette)
    app.setStyleSheet("QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }")


if __name__ == "__main__":
    application = QApplication([])
    set_style(application)
    rospy.init_node("MainWindow", disable_signals=True)
    main_window = MainWindow()
    main_window.show()
    application.exec_()
