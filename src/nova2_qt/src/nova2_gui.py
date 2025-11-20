# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'nova2_gui.ui'
##
## Created by: Qt User Interface Compiler version 6.8.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QDial, QDialog, QLabel,
    QPushButton, QSizePolicy, QVBoxLayout, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(382, 419)
        self.pushButton_testarm = QPushButton(Dialog)
        self.pushButton_testarm.setObjectName(u"pushButton_testarm")
        self.pushButton_testarm.setGeometry(QRect(90, 330, 201, 26))
        self.layoutWidget = QWidget(Dialog)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(60, 20, 258, 298))
        self.verticalLayout_2 = QVBoxLayout(self.layoutWidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label = QLabel(self.layoutWidget)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.label)

        self.dial_1 = QDial(self.layoutWidget)
        self.dial_1.setObjectName(u"dial_1")
        self.dial_1.setMaximum(360)
        self.dial_1.setWrapping(True)

        self.verticalLayout.addWidget(self.dial_1)

        self.dial_2 = QDial(self.layoutWidget)
        self.dial_2.setObjectName(u"dial_2")
        self.dial_2.setMinimum(0)
        self.dial_2.setMaximum(360)
        self.dial_2.setPageStep(0)
        self.dial_2.setTracking(True)
        self.dial_2.setOrientation(Qt.Orientation.Vertical)
        self.dial_2.setWrapping(True)
        self.dial_2.setNotchTarget(1.000000000000000)
        self.dial_2.setNotchesVisible(False)

        self.verticalLayout.addWidget(self.dial_2)

        self.dial_3 = QDial(self.layoutWidget)
        self.dial_3.setObjectName(u"dial_3")
        self.dial_3.setMaximum(360)
        self.dial_3.setOrientation(Qt.Orientation.Vertical)
        self.dial_3.setWrapping(False)

        self.verticalLayout.addWidget(self.dial_3)

        self.dial_4 = QDial(self.layoutWidget)
        self.dial_4.setObjectName(u"dial_4")
        self.dial_4.setMaximum(360)
        self.dial_4.setOrientation(Qt.Orientation.Vertical)
        self.dial_4.setWrapping(False)

        self.verticalLayout.addWidget(self.dial_4)


        self.verticalLayout_2.addLayout(self.verticalLayout)


        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Dialog", None))
        self.pushButton_testarm.setText(QCoreApplication.translate("Dialog", u"test robotarm", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"sliding cube", None))
    # retranslateUi

