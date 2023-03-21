from PyQt5.Qt import Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication


def key_pressed(obj, event):
    # If Ctrl+W is pressed - close the window
    if (QApplication.keyboardModifiers() == Qt.ControlModifier) and (event.key() == Qt.Key_W):
        obj.close()


def set_icon(obj):
    # Set window icon
    obj.setWindowIcon(QIcon('D:/Other/Desktop/Ucheba/Happrinn.ico'))
