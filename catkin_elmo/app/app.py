#! /usr/bin/env python


import sys

from PyQt5.QtCore import (
    Qt,
    QRunnable,
    pyqtSlot,
    pyqtSignal,
    QThreadPool,
    QObject,
    QEvent,
    QTimer,
    QUrl
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QDialog, QMessageBox, QVBoxLayout, QLabel,
    QDialogButtonBox, QPushButton,
    QFileDialog
)
from PyQt5.QtGui import QColor


from main_window_ui import Ui_MainWindow

import robot_client
import requests


class ScanRobotsWorker(QRunnable):

    class Signals(QObject):
        new_robot = pyqtSignal(str, str)
    
    def __init__(self, window):
        super().__init__()
        self.window = window
        self.signals = ScanRobotsWorker.Signals()

    @pyqtSlot()
    def run(self):
        self.window.label.setText("Scanning network for robots...")
        def cb(robot_name, robot_address):
            self.signals.new_robot.emit(robot_name, robot_address)
        robot_client.scan_robots(cb)


class ScanRobotsDialog(QDialog):

    def __init__(self, window, connect):
        super().__init__(parent=window)
        self.connect = connect
        self.setWindowTitle("Scan Robots")
        layout = QVBoxLayout()

        robots_container = QWidget()
        self.clients = QVBoxLayout()
        robots_container.setLayout(self.clients)
        self.client_names = []

        button_box = QDialogButtonBox(QDialogButtonBox.Cancel)
        button_box.rejected.connect(self.reject)

        self.label = QLabel()
        layout.addWidget(self.label)
        layout.addWidget(robots_container)
        layout.addWidget(button_box)

        self.setLayout(layout)

        self.tp = QThreadPool()
        scan_robots_worker = ScanRobotsWorker(self)
        scan_robots_worker.signals.new_robot.connect(self.on_new_robot)
        self.tp.start(scan_robots_worker)

    @pyqtSlot(str, str)
    def on_new_robot(self, name, address):
        if name not in self.client_names:
            btn = QPushButton(name + ": " + address)
            btn.clicked.connect(lambda: self.connect(address))
            self.clients.addWidget(btn)
            self.client_names.append(name)


class Window(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.show()
        self.setWindowTitle("Elmo")

        self.initialize_touch()
        self.initialize_leds()
        self.initialize_motors()
        self.initialize_behaviours()
        self.initialize_audio()
        self.initialize_screen()
        self.initialize_multimedia()
        self.initialize_wifi()
        self.log("Application running.")

        # scan robots on startup
        robot_client.set_robot_model("elmo")
        self.client = None
        self.scan_network.clicked.connect(self.scan_robots)
        self.shutdown.clicked.connect(self.do_shutdown)
        self.scan_robots()
        self.update()

    def log(self, msg, duration=0):
        self.status_bar.showMessage(msg, duration)

    def disconnect(self):
        self.client = None
        QMessageBox.warning(self, "Disconnect", "Connection to robot lost!")
        self.scan_robots()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W and not event.isAutoRepeat():
            self.is_painting = True
        elif event.key() == Qt.Key_C and not event.isAutoRepeat():
            self.is_clearing = True

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_W and not event.isAutoRepeat():
            self.is_painting = False
            self.send_colors()
        elif event.key() == Qt.Key_C and not event.isAutoRepeat():
            self.is_clearing = False
            self.send_colors()

    def scan_robots(self):
        dialog = ScanRobotsDialog(self, self.connect)
        self.dialog = dialog
        dialog.exec_()
        robot_client.stop_scan()

    def do_shutdown(self):
        if self.client is not None:
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Shutdown?", buttons=QMessageBox.Ok | QMessageBox.Cancel):
                self.client.send_command("shutdown")

    def connect(self, address):
        success, message, self.client = robot_client.connect(address)
        if success:
            self.client.on_error = self.log
            self.client.on_disconnect = self.disconnect
            self.dialog.close()
            self.log("Connected to robot at %s" % address)
        else:
            QMessageBox.warning(self, "Error", message)

    def send_colors(self):
        leds = []
        for row in range(13):
            for col in range(13):
                led = self.leds_matrix.itemAtPosition(row, col).widget()
                palette = led.palette().color(self.backgroundRole())
                leds.append((
                    palette.red(),
                    palette.green(),
                    palette.blue(),
                ))
        self.client.send_command("update_leds", colors=leds)

    def initialize_touch(self):
        def update_thresholds():
            if self.client is not None:
                if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Update touch threshold?", buttons=QMessageBox.Ok | QMessageBox.Cancel):
                    self.client.send_command(
                        "update_touch_thresholds",
                        head=self.touch_set_head_threshold.value(),
                        chest=self.touch_set_chest_threshold.value())
        self.touch_update_threshold.clicked.connect(update_thresholds)

    def initialize_leds(self):
        self.icon_list = []
        def update_icon():
            name = self.leds_icon_list.currentText()
            self.client.send_command("update_leds_icon", name=name)
        self.leds_icon_update.clicked.connect(update_icon)
        
        # color picker
        self.leds_preview.setAutoFillBackground(True)
        def update_preview():
            palette = self.leds_preview.palette()
            palette.setColor(self.backgroundRole(), QColor(self.leds_r.value(), self.leds_g.value(), self.leds_b.value()))
            self.leds_preview.setPalette(palette)
        self.leds_r.valueChanged.connect(update_preview)
        self.leds_g.valueChanged.connect(update_preview)
        self.leds_b.valueChanged.connect(update_preview)
        update_preview()
        # painting and led matrix initialization
        self.is_painting = False
        self.is_clearing = False
        def paint(led):
            def f(_):
                palette = led.palette()
                if palette.color(self.backgroundRole()) != Qt.black:
                    palette.setColor(self.backgroundRole(), Qt.black)
                else:
                    palette.setColor(self.backgroundRole(), QColor(self.leds_r.value(), self.leds_g.value(), self.leds_b.value()))
                led.setPalette(palette)
                self.send_colors()
            return f
        def fast_paint(led):
            def f(_):
                if self.is_painting:
                    palette = led.palette()
                    palette.setColor(self.backgroundRole(), QColor(self.leds_r.value(), self.leds_g.value(), self.leds_b.value()))
                    led.setPalette(palette)
                elif self.is_clearing:
                    palette = led.palette()
                    palette.setColor(self.backgroundRole(), Qt.black)
                    led.setPalette(palette)
            return f
        for row in range(13):
            for col in range(13):
                led = QWidget()
                led.setAutoFillBackground(True)
                palette = self.palette()
                palette.setColor(self.backgroundRole(), Qt.black)
                led.setPalette(palette)
                led.mousePressEvent = paint(led)                
                self.leds_matrix.addWidget(led, row, col)
                led.setMouseTracking(True)
                led.mouseMoveEvent = fast_paint(led)
        # paint shortcuts
        def paint_all():
            for row in range(13):
                for col in range(13):
                    led = self.leds_matrix.itemAtPosition(row, col).widget()
                    palette = led.palette()
                    palette.setColor(self.backgroundRole(), QColor(self.leds_r.value(), self.leds_g.value(), self.leds_b.value()))
                    led.setPalette(palette)
            self.send_colors()
        self.leds_all.clicked.connect(paint_all)
        def clear_all():
            for row in range(13):
                for col in range(13):
                    led = self.leds_matrix.itemAtPosition(row, col).widget()
                    palette = led.palette()
                    palette.setColor(self.backgroundRole(), Qt.black)
                    led.setPalette(palette)
            self.send_colors()
        self.leds_none.clicked.connect(clear_all)

    def initialize_motors(self):
        def update_motor_limits():
            def f():
                if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Update motor limits?", buttons=QMessageBox.Ok | QMessageBox.Cancel):
                    self.client.send_command(
                        "update_motor_limits",
                        pan_min=self.motors_set_pan_min.value(),
                        pan_max=self.motors_set_pan_max.value(),
                        tilt_min=self.motors_set_tilt_min.value(),
                        tilt_max=self.motors_set_tilt_max.value()
                    )
            return f
        self.motors_update_limits.clicked.connect(update_motor_limits())
        self.motors_pan.sliderReleased.connect(lambda: self.client.send_command("set_pan", angle=self.motors_pan.value()))
        self.motors_tilt.sliderReleased.connect(lambda: self.client.send_command("set_tilt", angle=self.motors_tilt.value()))
        self.motors_pan_torque_on.clicked.connect(lambda: self.client.send_command("set_pan_torque", control=True))
        self.motors_pan_torque_off.clicked.connect(lambda: self.client.send_command("set_pan_torque", control=False))
        self.motors_tilt_torque_on.clicked.connect(lambda: self.client.send_command("set_tilt_torque", control=True))
        self.motors_tilt_torque_off.clicked.connect(lambda: self.client.send_command("set_tilt_torque", control=False))

    def initialize_behaviours(self):
        def enable_test_motors(checked):
            self.client.send_command("enable_behaviour", name="test_motors", control=checked)
        self.behaviour_test_motors.stateChanged.connect(enable_test_motors)
        def enable_test_leds(checked):
            self.client.send_command("enable_behaviour", name="test_leds", control=checked)
        self.behaviour_test_leds.stateChanged.connect(enable_test_leds)

    def initialize_audio(self):
        self.speech_list = []
        self.sound_list = []
        def play_sound():
            self.client.send_command("play_sound", name=self.audio_sound_list.currentText())
        self.audio_play_sound.clicked.connect(play_sound)
        def play_speech():
            self.client.send_command("play_speech", name=self.audio_speech_list.currentText())
        self.audio_play_speech.clicked.connect(play_speech)
        def pause_audio():
            self.client.send_command("pause_audio")
        self.audio_pause.clicked.connect(pause_audio)
        def set_volume():
            self.client.send_command("set_volume", volume=self.audio_volume.value())
        self.audio_volume.sliderReleased.connect(set_volume)

    def initialize_screen(self):
        self.image_list = []
        def clear_screen():
            self.client.send_command(
                "set_screen",
                image="",
                text="",
                url="",
                camera=False
            )
        def update_screen():
            def f():
                self.client.send_command(
                    "set_screen",
                    image="" if self.screen_image_list.currentText() == "<None>" else self.screen_image_list.currentText(),
                    text=self.screen_text.text(),
                    url=self.screen_url.text(),
                    camera=self.screen_camera.isChecked(),
                )
            return f
        self.screen_clear.clicked.connect(clear_screen)
        self.screen_update.clicked.connect(update_screen())

    def initialize_multimedia(self):
        def upload_image():
            filename, _ = QFileDialog.getOpenFileName(self, 'Upload Image', '.')
            if filename:
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.image_address
                requests.post(url, files={'file': open(filename, 'rb')})
        self.multimedia_image_upload.clicked.connect(upload_image)
        def upload_icon():
            filename, _ = QFileDialog.getOpenFileName(self, 'Upload Icon', '.')
            if filename:
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.icon_address
                requests.post(url, files={'file': open(filename, 'rb')})
        self.multimedia_icon_upload.clicked.connect(upload_icon)
        def upload_speech():
            filename, _ = QFileDialog.getOpenFileName(self, 'Upload Speech', '.')
            if filename:
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.speech_address
                requests.post(url, files={'file': open(filename, 'rb')})
        self.multimedia_speech_upload.clicked.connect(upload_speech)
        def upload_sound():
            filename, _ = QFileDialog.getOpenFileName(self, 'Upload Sound', '.')
            if filename:
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.sound_address
                requests.post(url, files={'file': open(filename, 'rb')})
        self.multimedia_sound_upload.clicked.connect(upload_sound)
        def delete_image():
            filename = self.multimedia_image_list.currentText()
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Delete image %s?" % filename, buttons=QMessageBox.Ok | QMessageBox.Cancel):
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.image_address + "/" + filename
                print("delete: " + url)
                requests.delete(url)
        self.multimedia_image_delete.clicked.connect(delete_image)
        def delete_icon():
            filename = self.multimedia_icon_list.currentText()
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Delete icon %s?" % filename, buttons=QMessageBox.Ok | QMessageBox.Cancel):
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.icon_address + "/" + filename
                print("delete: " + url)
                requests.delete(url)
        self.multimedia_icon_delete.clicked.connect(delete_icon)
        def delete_speech():
            filename = self.multimedia_speech_list.currentText()
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Delete speech %s?" % filename, buttons=QMessageBox.Ok | QMessageBox.Cancel):
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.speech_address + "/" + filename
                print("delete: " + url)
                requests.delete(url)
        self.multimedia_speech_delete.clicked.connect(delete_speech)
        def delete_sound():
            filename = self.multimedia_sound_list.currentText()
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Delete sound %s?" % filename, buttons=QMessageBox.Ok | QMessageBox.Cancel):
                url = "http://" + self.client.ip + ":" + str(self.client.multimedia_port) + self.client.sound_address + "/" + filename
                print("delete: " + url)
                requests.delete(url)
        self.multimedia_sound_delete.clicked.connect(delete_sound)

    def initialize_wifi(self):
        def update_wifi_credentials():
            if QMessageBox.Ok == QMessageBox.warning(self, "Confirm", "Update wifi credentials?", buttons=QMessageBox.Ok | QMessageBox.Cancel):
                ssid = self.wifi_ssid.text()
                password = self.wifi_password.text()
                print("updating credentials")
                if self.client.send_command("update_wifi_credentials", ssid=ssid, password=password):
                    QMessageBox.information(self, "Configuration updated", "Changes will take effect after a restart")
        self.wifi_update.clicked.connect(update_wifi_credentials)

    def update(self):
        if self.client is not None:
            self.client.update_status()
            self.motors_pan.setRange(self.client.pan_min, self.client.pan_max)
            self.motors_pan_min.setNum(self.client.pan_min)
            self.motors_pan_max.setNum(self.client.pan_max)
            self.motors_pan_value.setNum(self.client.pan)
            self.motors_pan_torque.setChecked(self.client.pan_torque)
            self.motors_tilt.setRange(self.client.tilt_min, self.client.tilt_max)
            self.motors_tilt_min.setNum(self.client.tilt_min)
            self.motors_tilt_max.setNum(self.client.tilt_max)
            self.motors_tilt_value.setNum(self.client.tilt)
            self.motors_tilt_torque.setChecked(self.client.tilt_torque)
            self.touch_chest.setChecked(self.client.touch_chest)
            self.touch_head_n.setChecked(self.client.touch_head_n)
            self.touch_head_s.setChecked(self.client.touch_head_s)
            self.touch_head_e.setChecked(self.client.touch_head_e)
            self.touch_head_w.setChecked(self.client.touch_head_w)
            self.touch_head_threshold.setNum(self.client.touch_head_threshold)
            self.touch_chest_threshold.setNum(self.client.touch_chest_threshold)
            if self.client.icon_list != self.icon_list:
                self.leds_icon_list.clear()
                self.leds_icon_list.addItems(self.client.icon_list)
                self.multimedia_icon_list.clear()
                self.multimedia_icon_list.addItems(self.client.icon_list)
                self.icon_list = self.client.icon_list
            if self.client.speech_list != self.speech_list:
                self.audio_speech_list.clear()
                self.audio_speech_list.addItems(self.client.speech_list)
                self.multimedia_speech_list.clear()
                self.multimedia_speech_list.addItems(self.client.speech_list)
                self.speech_list = self.client.speech_list
            if self.client.sound_list != self.sound_list:
                self.audio_sound_list.clear()
                self.audio_sound_list.addItems(self.client.sound_list)
                self.multimedia_sound_list.clear()
                self.multimedia_sound_list.addItems(self.client.sound_list)
                self.sound_list = self.client.sound_list
            if self.client.image_list != self.image_list:
                self.screen_image_list.clear()
                self.screen_image_list.addItems(["<None>"] + self.client.image_list)
                self.multimedia_image_list.clear()
                self.multimedia_image_list.addItems(self.client.image_list)
                self.image_list = self.client.image_list

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(100)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    app.exec()
    sys.exit()
