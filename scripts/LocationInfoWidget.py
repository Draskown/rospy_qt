from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator
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

        self.initUI()

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
        self.x_start_input.setValidator(QIntValidator())
        self.y_start_input = QLineEdit(self)
        self.y_start_input.setValidator(QIntValidator())
        set_position_btn = QPushButton("Set position")
        reset_position_btn = QPushButton("Reset position")

        # Create text outputs for correction and position
        self.correction_of_direction_output = QLabel("0")
        self.correction_of_position_output = QLabel("0")
        self.global_position = QLabel("0, 0")

        # Create Text outputs for the encoders and direction
        self.encoder1 = QLabel("0")
        self.encoder2 = QLabel("0")
        self.direction = QLabel("0, 0")

        # Configure an array of labels
        labels = {"x_start": QLabel("X start position:"),
                  "y_start": QLabel("Y start position:"),
                  "cor_dir_label": QLabel("Correction of \n direction:"),
                  "cor_dir": self.correction_of_direction_output,
                  "cor_pos_label": QLabel("Correction of \n position:"),
                  "cor_pos": self.correction_of_position_output,
                  "pos_label": QLabel("Global position:"),
                  "pos": self.global_position,
                  "enc1_label": QLabel("Encoder1:"),
                  "enc1": self.encoder1,
                  "enc2_label": QLabel("Encoder2:"),
                  "enc2": self.encoder2,
                  "dir_label": QLabel("Direction"),
                  "dir": self.direction}

        for item in labels.items():
            item[1].setAlignment(Qt.AlignHCenter)

        # Add Items to the first column
        column1 = QVBoxLayout()

        column1.addWidget(labels["x_start"], Qt.AlignHCenter)
        column1.addWidget(self.x_start_input, Qt.AlignHCenter)
        column1.addWidget(labels["y_start"], Qt.AlignHCenter)
        column1.addWidget(self.y_start_input, Qt.AlignHCenter)
        column1.addWidget(set_position_btn, Qt.AlignHCenter)
        column1.addWidget(reset_position_btn, Qt.AlignHCenter)

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
        column3.addWidget(self.encoder1, Qt.AlignHCenter)
        column3.addWidget(labels["enc2_label"], Qt.AlignHCenter)
        column3.addWidget(self.encoder2, Qt.AlignHCenter)
        column3.addWidget(labels["dir_label"], Qt.AlignHCenter)
        column3.addWidget(self.direction, Qt.AlignHCenter)

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
