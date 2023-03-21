from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget,
                             QLabel,
                             QVBoxLayout,
                             QApplication)
from PyQt5.QtGui import QPixmap
from Common_defs import set_icon, key_pressed


class LidarWidget(QWidget):
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
        # Set Window Name
        self.setWindowTitle("Lidar Widget")

        # Create QLabel for image output
        self.image_label = QLabel()

        # Set an icon
        set_icon(self)

        # Set an image
        self.set_image("./Images/2.jpg")

        # Create QVBoxLayout for layout
        layout = QVBoxLayout()

        # Add image label to layout
        layout.addWidget(self.image_label, alignment=Qt.AlignCenter)

        # Set layout for widget
        self.setLayout(layout)

    def set_image(self, path):
        # Convert image to pixmap and set it as image label's pixmap
        pixmap = QPixmap(path).scaled(600, 400)
        self.image_label.setPixmap(pixmap)
    ###