import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

PKG = "rqt_gimbal_control"
UI = "gimbal_control.ui"

class UIWidget(QWidget):
    def __init__(self):
        super(UIWidget, self).__init__()
        _, package_path = get_resource('packages', PKG)
        ui_file = os.path.join(package_path, 'share', PKG, 'resource', UI)
        loadUi(ui_file, self)
        