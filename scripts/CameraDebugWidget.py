from PyQt5.Qt import Qt
from PyQt5.QtWidgets import (QWidget,
                             QLabel,
                             QVBoxLayout,
                             QHBoxLayout,
                             QGridLayout,
                             QComboBox,
                             QCheckBox,
                             QSizePolicy,
                             QApplication)
from PyQt5.QtGui import QPixmap
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

        # Create image output
        self.image_label = QLabel()
        self.image_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

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

        Grid_combo.addWidget(QLabel('Front Camera Mode:'), 0, 0,
                             alignment=Qt.AlignCenter)
        Grid_combo.addWidget(self.front_camera_combo, 1, 0)
        Grid_combo.addWidget(QLabel('Line Camera Mode:'), 0, 1,
                             alignment=Qt.AlignCenter)
        Grid_combo.addWidget(self.line_camera_combo, 1, 1)

        HLayout_switch.addWidget(environment_switch)
        HLayout_switch.addWidget(line_detection_switch)
        HLayout_switch.addWidget(perspective_switch)

        # Create main layout for widget
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.image_label, alignment=Qt.AlignHCenter)
        main_layout.addLayout(Grid_combo)
        main_layout.addLayout(HLayout_switch)

        self.setLayout(main_layout)

        self.set_image("./Images/3.jpg")

    def set_image(self, path):
        # Convert image to pixmap and set it as image label's pixmap
        pixmap = QPixmap(path).scaled(600, 400)
        self.image_label.setPixmap(pixmap)
    ###
