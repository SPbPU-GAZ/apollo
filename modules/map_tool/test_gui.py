#!/usr/bin/python3
import os
import sys
from PyQt5.QtWidgets import QApplication

from application.MainWindow import MainWindow
from application.Scenario import Scenario


# if not os.path.exists(cfg.TMP_DIR):
#     os.mkdir(cfg.TMP_DIR)

scenario = Scenario()

application = QApplication(sys.argv)
window = MainWindow(scenario)
window.show()
application.exec()