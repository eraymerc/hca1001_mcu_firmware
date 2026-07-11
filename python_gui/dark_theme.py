"""Dark theme: a Qt stylesheet plus matching pyqtgraph global colors."""

import pyqtgraph as pg

DARK_QSS = """
QWidget {
    background-color: #1e1e1e;
    color: #e0e0e0;
    font-size: 10.5pt;
}
QMainWindow, QDialog {
    background-color: #1e1e1e;
}
QGroupBox {
    border: 1px solid #3a3a3a;
    border-radius: 4px;
    margin-top: 10px;
    padding-top: 6px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: #9fd3ff;
}
QPushButton {
    background-color: #2d2d2d;
    border: 1px solid #454545;
    border-radius: 4px;
    padding: 5px 12px;
}
QPushButton:hover {
    background-color: #3a3a3a;
    border-color: #5a5a5a;
}
QPushButton:pressed {
    background-color: #454545;
}
QPushButton:disabled {
    color: #6a6a6a;
    background-color: #262626;
}
QPushButton#dangerButton {
    background-color: #5a2323;
    border-color: #7a3030;
}
QPushButton#dangerButton:hover {
    background-color: #6e2b2b;
}
QComboBox, QLineEdit {
    background-color: #2a2a2a;
    border: 1px solid #454545;
    border-radius: 4px;
    padding: 3px 6px;
}
QComboBox QAbstractItemView {
    background-color: #2a2a2a;
    selection-background-color: #3a5a7a;
}
QCheckBox {
    spacing: 6px;
}
QLabel#statusLabel {
    color: #9fd3ff;
    font-weight: 600;
}
QLabel#statusLabelOk {
    color: #6fe08a;
    font-weight: 600;
}
QLabel#statusLabelError {
    color: #ff8080;
    font-weight: 600;
}
QStatusBar {
    background-color: #181818;
    border-top: 1px solid #3a3a3a;
}
"""


def apply_dark_theme(app):
    app.setStyleSheet(DARK_QSS)
    pg.setConfigOption("background", "#1e1e1e")
    pg.setConfigOption("foreground", "#c8c8c8")
    pg.setConfigOption("antialias", True)
