#!/usr/bin/env python3
import json,sys
from pathlib import Path
from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QLabel
from PyQt5.QtCore import QTimer,Qt
app=QApplication(sys.argv);window=QWidget();window.setWindowTitle('Trial observations');window.setWindowFlags(window.windowFlags()|Qt.WindowStaysOnTopHint)
layout=QVBoxLayout(window);label=QLabel('Waiting for trial');label.setStyleSheet('font-size: 18px');layout.addWidget(label)
path=Path(sys.argv[1])
def refresh():
    try:
        d=json.loads(path.read_text());heading='Review' if path.parent.name=='review' else f"Test {d['test']:02}";label.setText(f"{heading}   |   {d['speed']:g} m/s   |   ±{d['angle']}°   |   obstacle {d['distance']:g} m   |   {d['elapsed']:.1f} / 30 s\nCollisions: {d['collisions']}   (Gazebo contact sensors)")
    except (OSError,ValueError):pass
timer=QTimer();timer.timeout.connect(refresh);timer.start(200);window.resize(850,90);window.move(520,940);window.show();app.exec_()
