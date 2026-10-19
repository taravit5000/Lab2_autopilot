#!/usr/bin/env python3
"""Save the rqt_graph window."""
import sys
from pathlib import Path
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QComboBox, QPushButton, QGraphicsView, QLineEdit, QCheckBox
from rqt_gui.main import Main
out=Path(sys.argv[1]).resolve()
class CaptureMain(Main):
    def create_application(self, argv):
        app=super().create_application(argv)
        def setup():
            for widget in app.topLevelWidgets():
                if widget.findChild(QComboBox,'graph_type_combo_box'):
                    widget.resize(1600,950)
                    widget.findChild(QComboBox,'graph_type_combo_box').setCurrentIndex(2)
                    if '--full' not in sys.argv:
                        widget.findChild(QLineEdit,'filter_line_edit').setText('/autopilot,/turtlebot3_laserscan,/turtlebot3_diff_drive,/slam_toolbox,/rviz2')
                        widget.findChild(QLineEdit,'topic_filter_line_edit').setText('/scan,/cmd_vel,/odom,/map')
                        widget.findChild(QCheckBox,'leaf_topics_check_box').setChecked(False)
                        widget.findChild(QCheckBox,'dead_sinks_check_box').setChecked(False)
                        widget.findChild(QCheckBox,'quiet_check_box').setChecked(False)
                    widget.findChild(QPushButton,'refresh_graph_push_button').click()
        def capture():
            for widget in app.topLevelWidgets():
                if widget.findChild(QComboBox,'graph_type_combo_box'):
                    widget.findChild(QPushButton,'fit_in_view_push_button').click()
                    views=widget.findChildren(QGraphicsView)
                    n=sum(len(v.scene().items()) for v in views if v.scene())
                    if n < 10: raise RuntimeError('rqt graph scene is empty')
                    if not widget.grab().save(str(out)): raise RuntimeError('capture failed')
                    print(f'Captured actual rqt_graph window: {n} scene items',flush=True)
            for widget in app.topLevelWidgets():
                widget.close()
        QTimer.singleShot(4000,setup)
        QTimer.singleShot(7500,capture)
        return app
sys.exit(CaptureMain().main(['rqt_graph','--clear-config'], standalone='rqt_graph.ros_graph.RosGraph'))
