import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import (QWidget,
                             QPushButton,
                             QGridLayout,
                             QApplication)
from Common_defs import set_icon, key_pressed


class ManualMovementWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.init_UI()

        self.btn_pub = rospy.Publisher('teleop_btn', String, queue_size=1)

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
        self.setWindowTitle("Manual Movement Widget")

        # Set an icon
        set_icon(self)

        # Create buttons
        forward_button = QPushButton('Forward')
        turn_left_button = QPushButton('Turn Left')
        stop_rotation_button = QPushButton('Stop Rotation')
        stop_button = QPushButton('Stop')
        turn_right_button = QPushButton('Turn Right')
        backward_button = QPushButton('Backward')

        # Connect buttons
        forward_button.clicked.connect(lambda: self.btn_pushed("w"))
        turn_left_button.clicked.connect(lambda: self.btn_pushed("a"))
        stop_rotation_button.clicked.connect(lambda: self.btn_pushed("s_r"))
        stop_button.clicked.connect(lambda: self.btn_pushed(" "))
        turn_right_button.clicked.connect(lambda: self.btn_pushed("d"))
        backward_button.clicked.connect(lambda: self.btn_pushed("s"))

        # Create grid layout
        grid = QGridLayout()

        # Add buttons to grid layout
        grid.addWidget(forward_button, 0, 1)
        grid.addWidget(turn_left_button, 1, 0)
        grid.addWidget(stop_rotation_button, 1, 1)
        grid.addWidget(stop_button, 2, 1)
        grid.addWidget(turn_right_button, 1, 3)
        grid.addWidget(backward_button, 3, 1)

        # Set layout for widget
        self.setLayout(grid)

    
    def btn_pushed(self, btn):
        self.btn_pub.publish(btn)