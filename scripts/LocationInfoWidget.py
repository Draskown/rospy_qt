import rospy, numpy as np
from std_msgs.msg import Float64
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (QWidget,
                             QVBoxLayout,
                             QHBoxLayout,
                             QLabel,
                             QLineEdit,
                             QPushButton,
                             QApplication)
from Common_defs import set_icon, key_pressed


class LocationInfoWidget(QWidget):
    def __init__(self):
        super().__init__()

        # init global variables
        self.position = [0.0, 0.0]

        self.initUI()

        # Set publishers for ROS topics
        self.position_x_pub = rospy.Publisher('set_x', Float64, queue_size=1)
        self.position_y_pub = rospy.Publisher('set_y', Float64, queue_size=1)
        self.direction_pub = rospy.Publisher('set_dir', Float64, queue_size=1)

        # Subscribers for the positions
        x_sub = rospy.Subscriber('position_x', Float64, self.set_x, queue_size=1)
        y_sub = rospy.Subscriber('position_y', Float64, self.set_y, queue_size=1)
        direction_sub = rospy.Subscriber('direction', Float64, self.set_dir, queue_size=1)
        enc_r_sub = rospy.Subscriber('enc_r', Float64, self.set_right, queue_size=1)
        enc_l_sub = rospy.Subscriber('enc_l', Float64, self.set_left, queue_size=1)
        pub_cor_sub = rospy.Subscriber('cor_pos', Float64, self.set_cor_pos, queue_size=1)
        dir_cor_sub = rospy.Subscriber('cor_dir', Float64, self.set_cor_dir, queue_size=1)

    # Events
    def keyPressEvent(self, e):
        key_pressed(self, e)

    # Clear focus if mouse pressed on an empty space
    def mousePressEvent(self, e):
        if QApplication.focusWidget() is not None:
            QApplication.focusWidget().clearFocus()
    ###

    # Methods
    def initUI(self):
        # Set Window Name
        self.setWindowTitle("Location info Widget")

        # Set an icon
        set_icon(self)

        # Create user input fields
        main_layout = QHBoxLayout()

        # Create user inputs and buttons for setting the position
        self.x_start_input = QLineEdit(self)
        self.x_start_input.setValidator(QDoubleValidator())
        self.y_start_input = QLineEdit(self)
        self.y_start_input.setValidator(QDoubleValidator())
        self.set_position_btn = QPushButton("Set position")
        self.reset_position_btn = QPushButton("Reset position")

        self.set_position_btn.clicked.connect(self.set_position)
        self.reset_position_btn.clicked.connect(self.reset_position)

        # Create text outputs for correction and position
        self.correction_of_direction_output = QLabel("0")
        self.correction_of_position_output = QLabel("0")
        self.global_position = QLabel()

        # Create Text outputs for the encoders and direction
        self.right_encoder = QLabel()
        self.left_encoder = QLabel()
        self.direction_label = QLabel()

        # Configure an array of labels
        labels = {"x_start": QLabel("X start position:"),
                  "y_start": QLabel("Y start position:"),
                  "cor_dir_label": QLabel("Correction of \n direction:"),
                  "cor_dir": self.correction_of_direction_output,
                  "cor_pos_label": QLabel("Correction of \n position:"),
                  "cor_pos": self.correction_of_position_output,
                  "pos_label": QLabel("Global position:"),
                  "pos": self.global_position,
                  "enc1_label": QLabel("Right Encoder:"),
                  "enc1": self.right_encoder,
                  "enc2_label": QLabel("Left Encoder:"),
                  "enc2": self.left_encoder,
                  "dir_label": QLabel("Direction"),
                  "dir": self.direction_label}

        for item in labels.items():
            item[1].setAlignment(Qt.AlignHCenter)

        # Add Items to the first column
        column1 = QVBoxLayout()

        column1.addWidget(labels["x_start"], Qt.AlignHCenter)
        column1.addWidget(self.x_start_input, Qt.AlignHCenter)
        column1.addWidget(labels["y_start"], Qt.AlignHCenter)
        column1.addWidget(self.y_start_input, Qt.AlignHCenter)
        column1.addWidget(self.set_position_btn, Qt.AlignHCenter)
        column1.addWidget(self.reset_position_btn, Qt.AlignHCenter)

        # Add Items to the second column
        column2 = QVBoxLayout()

        column2.addWidget(labels["cor_dir_label"], Qt.AlignHCenter)
        column2.addWidget(self.correction_of_direction_output, Qt.AlignHCenter)
        column2.addWidget(labels["cor_pos_label"], Qt.AlignHCenter)
        column2.addWidget(self.correction_of_position_output, Qt.AlignHCenter)
        column2.addWidget(labels["pos_label"], Qt.AlignHCenter)
        column2.addWidget(self.global_position, Qt.AlignHCenter)

        # Add Items to the third column
        column3 = QVBoxLayout()
        column3.addWidget(labels["enc1_label"], Qt.AlignHCenter)
        column3.addWidget(self.right_encoder, Qt.AlignHCenter)
        column3.addWidget(labels["enc2_label"], Qt.AlignHCenter)
        column3.addWidget(self.left_encoder, Qt.AlignHCenter)
        column3.addWidget(labels["dir_label"], Qt.AlignHCenter)
        column3.addWidget(self.direction_label, Qt.AlignHCenter)

        # Add all of the columns into the Main Layout
        main_layout.addLayout(column1)
        main_layout.addLayout(column2)
        main_layout.addLayout(column3)

        # Set identical widths for all of the columns
        main_layout.setStretchFactor(column1, 4)
        main_layout.setStretchFactor(column2, 4)
        main_layout.setStretchFactor(column3, 4)

        self.setLayout(main_layout)

        # Set window properties
        self.setWindowTitle("Location Info Widget")

    # Listens fot the x position
    def set_x(self, data):
        self.position[0] = data.data
        self.update_position()

    # Listens for the y position
    def set_y(self, data):
        self.position[1] = data.data
        self.update_position()

    # Listens for the direction
    def set_dir(self, data):
        label_text = str(round(data.data, 3))
        self.direction_label.setText(label_text)

    # Updates the position label
    def update_position(self):
        label_text = str(round(self.position[0], 3)) + ", " + str(round(self.position[1], 3))
        self.global_position.setText(label_text)

    # Publish the set position of the encoders
    def set_position(self):
        x_pos = Float64()
        y_pos = Float64()
        x_pos.data = np.float(self.x_start_input.text())
        y_pos.data = np.float(self.y_start_input.text())
        self.position_x_pub.publish(x_pos)
        self.position_y_pub.publish(y_pos)
    
    # Publish the resetted position of the encoders
    def reset_position(self):
        x_pos = Float64()
        y_pos = Float64()
        x_pos.data = 0.0
        y_pos.data = 0.0
        self.x_start_input.setText("")
        self.y_start_input.setText("")
        self.position_x_pub.publish(x_pos)
        self.position_y_pub.publish(y_pos)
        self.direction_pub.publish(0.0)

    # Listens for the left encoder's value
    def set_left(self, data):
        label_text = str(round(data.data, 3))
        self.left_encoder.setText(label_text)

    # Listens for the right encoder's value
    def set_right(self, data):
        label_text = str(round(data.data, 3))
        self.right_encoder.setText(label_text)

    # Listens for the correction of the position
    def set_cor_pos(self, data):
        label_text = str(round(data.data, 3))
        self.correction_of_position_output.setText(label_text)

    # Listens for the correction of the direction
    def set_cor_dir(self, data):
        label_text = str(round(data.data, 3))
        self.correction_of_direction_output.setText(label_text)

