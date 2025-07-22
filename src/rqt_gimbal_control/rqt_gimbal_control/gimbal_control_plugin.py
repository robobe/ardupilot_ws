#!/usr/bin/env python3

from .gimbal_control_ui_loader import UIWidget
from rqt_gui_py.plugin import Plugin
from .gimbal_control_backend import BackendNode, RangeTranslate
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from PyQt5.QtWidgets import QVBoxLayout, QComboBox, QCheckBox, QLineEdit, QPushButton, QSlider
from PyQt5.QtCore import Qt  # Qt.Checked is here
from pymavlink.dialects.v20 import common as mavlink_common


ROLL_MIN , ROLL_MAX = -50, 50
PITCH_MIN, PITCH_MAX = -90, 90
YAW_MIN, YAW_MAX = -180, 180

class GimbalControl(Plugin):
    def __init__(self, context):
        super().__init__(context)

        self._widget = UIWidget()
        self._backend = BackendNode()
        self._roll_range = RangeTranslate(0, 100, ROLL_MIN, ROLL_MAX)
        self._yaw_range = RangeTranslate(0, 100, YAW_MIN, YAW_MAX)
        self._pitch_range = RangeTranslate(0, 100, PITCH_MIN, PITCH_MAX)

        self.slider_pitch: QSlider = self._widget.slider_pitch
        self.slider_pitch.sliderMoved.connect(self.on_pitch_changed)
        self.slider_pitch.sliderPressed.connect(self.slider_move_begin)
        self.slider_pitch.sliderReleased.connect(self.slider_move_end)

        self.slider_yaw: QSlider = self._widget.slider_yaw
        self.slider_yaw.sliderMoved.connect(self.on_yaw_changed)
        self.slider_yaw.sliderPressed.connect(self.slider_move_begin)
        self.slider_yaw.sliderReleased.connect(self.slider_move_end)

        self.slider_roll: QSlider = self._widget.slider_roll
        self.slider_roll.sliderMoved.connect(self.on_roll_changed)
        self.slider_roll.sliderPressed.connect(self.slider_move_begin)
        self.slider_roll.sliderReleased.connect(self.slider_move_end)

        self.cmd_retract: QPushButton = self._widget.cmd_retract
        self.cmd_retract.clicked.connect(self.on_retract_clicked)
        self.cmd_neutral: QPushButton = self._widget.cmd_neutral
        self.cmd_neutral.clicked.connect(self.on_neutral_clicked)
        self.cmd_preset: QPushButton = self._widget.cmd_preset
        self.cmd_preset.clicked.connect(self.on_preset_clicked)

        self.txt_roll: QLineEdit = self._widget.txt_roll
        self.txt_pitch: QLineEdit = self._widget.txt_pitch
        self.txt_yaw: QLineEdit = self._widget.txt_yaw

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.enable_feedback = True
        self._backend.on_gimbal_status += self.gimbal_status_handler
        context.add_widget(self._widget)



    def gimbal_status_handler(self, r, p, y):
        self.txt_roll.setText(str(round(r,2)))
        self.txt_pitch.setText(str(round(p,2)))
        self.txt_yaw.setText(str(round(y,2)))
        
        if not self.enable_feedback:
            return
        roll_value = int(self._roll_range.translate_reverse(r))
        pitch_value = int(self._yaw_range.translate_reverse(p))
        yaw_value = int(self._pitch_range.translate_reverse(y))

        self.slider_roll.setValue(roll_value)
        self.slider_pitch.setValue(pitch_value)
        self.slider_yaw.setValue(yaw_value)

    def slider_move_begin(self):
        self.enable_feedback = False

    def slider_move_end(self):
        self.enable_feedback = True

    def shutdown_plugin(self):
        self._backend.close()

    def on_retract_clicked(self):
        self.roll = 0
        self.yaw = 0
        self._backend.send_gimbal_command(0, 0, 0, mavlink_common.GIMBAL_MANAGER_FLAGS_RETRACT)

    def on_neutral_clicked(self):
        self.roll = 0
        self.yaw = 0
        self._backend.send_gimbal_command(0, 0, 0, mavlink_common.GIMBAL_MANAGER_FLAGS_NEUTRAL)

    def on_preset_clicked(self):
        self.roll = 0
        self.yaw = 0
        self.pitch = 70
        self._backend.send_gimbal_command(self.roll, self.pitch, self.yaw, mavlink_common.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
        
    def on_yaw_changed(self, value):
        self.yaw = self._yaw_range.translate(value)
        self._backend.send_gimbal_command(self.roll, self.pitch, self.yaw, mavlink_common.GIMBAL_MANAGER_FLAGS_YAW_LOCK)

    def on_roll_changed(self, value):
        self.roll = self._roll_range.translate(value)
        self._backend.send_gimbal_command(self.roll, self.pitch, self.yaw, mavlink_common.GIMBAL_MANAGER_FLAGS_YAW_LOCK)

    def on_pitch_changed(self, value):
        self.pitch = self._pitch_range.translate(value)
        self._backend.send_gimbal_command(self.roll, self.pitch, self.yaw, mavlink_common.GIMBAL_MANAGER_FLAGS_YAW_LOCK)

