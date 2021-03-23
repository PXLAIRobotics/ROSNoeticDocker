#!/usr/bin/env python3

from PyQt5 import QtGui, QtCore, QtWidgets
import cv2
import sys

class DisplayImageWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)

        self.button = QtWidgets.QPushButton('Pillars of Creation')
        self.button.clicked.connect(self.show_image)
        self.image_frame = QtWidgets.QLabel()

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    @QtCore.pyqtSlot()
    def show_image(self):
        opencv_image = cv2.imread('../Images/pillars_of_creation.jpg')
        rgb_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        self.image = QtGui.QImage(rgb_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(self.image))

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    display_image_widget = DisplayImageWidget()
    display_image_widget.show()
    sys.exit(app.exec_())
