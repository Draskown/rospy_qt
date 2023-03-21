from PyQt5.Qt import Qt
from PyQt5.QtGui import (QPalette,
                         QColor,
                         QPixmap,
                         QFont)
from PyQt5.QtWidgets import (QApplication,
                             QMainWindow,
                             QPushButton,
                             QLabel,
                             QWidget,
                             QHBoxLayout,
                             QVBoxLayout,
                             QMessageBox)

from CameraDebugWidget import CameraDebugWidget
from LidarWidget import LidarWidget
from LocationInfoWidget import LocationInfoWidget
from ManualMovementWidget import ManualMovementWidget
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

        # Create layout for central widget
        main_layout = QHBoxLayout()

        # Set image
        image_label = QLabel()
        image = QPixmap("./Images/1.png").scaled(640, 480)
        image_label.setPixmap(image)

        # Create Camera Debug button
        camera_debug_button = QPushButton('Camera Debug')

        # Create Lidar button
        lidar_button = QPushButton('Lidar')

        # Create Location Info button
        location_info_button = QPushButton('Location Info')

        # Create Manual Movement button
        manual_movement_button = QPushButton('Manual Movement')

        # Create Start Mission switch
        start_mission_button = QPushButton('Start Mission')

        # Add Items to the first column in a HLayout
        column1 = QVBoxLayout()
        column1.addStretch()
        column1.addWidget(camera_debug_button)
        column1.addWidget(lidar_button)
        column1.addStretch()

        # Add Items to the second column in a HLayout
        column2 = QVBoxLayout()
        column2.addWidget(image_label, Qt.AlignTop)
        column2.addWidget(start_mission_button, 5, Qt.AlignHCenter)

        # Add Items to the third column in a HLayout
        column3 = QVBoxLayout()
        column3.addStretch()
        column3.addWidget(location_info_button, Qt.AlignTop)
        column3.addWidget(manual_movement_button, Qt.AlignTop)
        column3.addStretch()

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
        manual_movement_button.clicked.connect(self.show_manual_movement_widget)

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
    def show_camera_debug_widget(self):
        self.camera_debug_widget.show()

    def show_lidar_widget(self):
        self.lidar_widget.show()

    def show_location_info_widget(self):
        self.location_info_widget.show()

    def show_manual_movement_widget(self):
        self.manual_movement_widget.show()
    ###


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
    main_window = MainWindow()
    main_window.show()
    application.exec_()
