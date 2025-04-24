# -*- coding: utf-8 -*-
import sys
import os
import math
import cv2
import serial
import numpy as np
from datetime import datetime, timezone
from serial.tools import list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSplitter, QStatusBar,
    QSplashScreen
)
from PyQt5.QtCore import QTimer, Qt, QDateTime
from PyQt5.QtGui import QPixmap, QImage

import motor
import imu
import spectrometer
import filterwheel
import utils
from temp_controller import TC36_25

import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1000, 750)

        # --- Thread management ---
        self._running_threads = []  # keep references to all QThreads

        # --- State variables ---
        self.motor_serial = None
        self.motor_connected = False
        self.motor_comm_issue = False
        self.filterwheel_serial = None
        self.filterwheel_connected = False
        self.filterwheel_comm_issue = False
        self.imu_serial = None
        self.imu_connected = False
        self.spec_handle = None
        self.measurement_active = False
        self.last_filter_command = None

        # --- Status bar ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # --- Main layout ---
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(8)

        # Logo
        logo_layout = QHBoxLayout()
        logo_layout.addStretch()
        logo = QLabel()
        pix = QPixmap("asset/sciglob_symbol.png")
        if not pix.isNull():
            logo.setPixmap(pix.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        logo_layout.addWidget(logo)
        main_layout.addLayout(logo_layout)

        # Temperature Control
        temp_layout = QHBoxLayout()
        temp_layout.setSpacing(10)
        temp_layout.addWidget(QLabel("Set Temp (°C):"))
        self.set_temp_input = QLineEdit("20.0")
        self.set_temp_input.setFixedWidth(60)
        temp_layout.addWidget(self.set_temp_input)
        self.send_temp_button = QPushButton("Send Cmd")
        self.send_temp_button.clicked.connect(self.on_send_temp)
        temp_layout.addWidget(self.send_temp_button)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(QLabel("Current Temp:"))
        self.current_temp_label = QLabel("-- °C")
        temp_layout.addWidget(self.current_temp_label)
        temp_layout.addStretch()
        main_layout.addLayout(temp_layout)

        # Initialize temperature controller
        try:
            self.tc = TC36_25()  # COM16 default
            self.tc.enable_computer_setpoint()
            self.tc.power(True)
            self.temp_timer = QTimer(self)
            self.temp_timer.timeout.connect(self.update_temperature)
            self.temp_timer.start(1000)
        except Exception as e:
            self.status_bar.showMessage(f"Temp ctrl init failed: {e}")

        # Spectrometer Group
        spectro_controls = QHBoxLayout()
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.setEnabled(False)
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        spectro_controls.addWidget(self.toggle_save_button)
        self.connect_spec_button = QPushButton("Connect Spectrometer")
        self.connect_spec_button.clicked.connect(self.on_connect_spectrometer)
        spectro_controls.addWidget(self.connect_spec_button)
        self.start_meas_button = QPushButton("Start Measurement")
        self.start_meas_button.setEnabled(False)
        self.start_meas_button.clicked.connect(self.on_start_measurement)
        spectro_controls.addWidget(self.start_meas_button)
        self.stop_meas_button = QPushButton("Stop")
        self.stop_meas_button.setEnabled(False)
        self.stop_meas_button.clicked.connect(self.on_stop_measurement)
        spectro_controls.addWidget(self.stop_meas_button)
        self.save_data_button = QPushButton("Save Data")
        self.save_data_button.setEnabled(False)
        self.save_data_button.clicked.connect(self.on_save_data)
        spectro_controls.addWidget(self.save_data_button)

        pg.setConfigOption('background','w')
        pg.setConfigOption('foreground','k')
        self.spec_plot = pg.PlotWidget()
        self.spec_plot.setLabel('bottom','Wavelength',units='nm')
        self.spec_plot.setLabel('left','Intensity',units='counts')
        self.spec_plot.showGrid(x=True,y=True,alpha=0.3)
        self.spec_plot.setXRange(260,600)
        self.spec_curve = self.spec_plot.plot([],[],pen=pg.mkPen('#2986cc',width=1))

        self.spectro_group = QGroupBox("Spectrometer")
        sp_layout = QVBoxLayout()
        sp_layout.addLayout(spectro_controls)
        sp_layout.addWidget(self.spec_plot,stretch=1)
        self.spectro_group.setLayout(sp_layout)

        # Populate COM port list
        ports = [getattr(p,'name',p.device) for p in list_ports.comports()]
        if not ports:
            ports = [f"COM{i}" for i in range(1,10)]

        # Motor Group
        motor_layout = QGridLayout()
        motor_layout.addWidget(QLabel("Motor COM:"),0,0)
        self.motor_port_combo = QComboBox()
        self.motor_port_combo.setEditable(True)
        self.motor_port_combo.addItems(ports)
        motor_layout.addWidget(self.motor_port_combo,0,1)
        self.motor_connect_button = QPushButton("Connect Motor")
        self.motor_connect_button.clicked.connect(self.on_connect_motor)
        motor_layout.addWidget(self.motor_connect_button,0,2)
        motor_layout.addWidget(QLabel("Angle (°):"),1,0)
        self.angle_input = QLineEdit()
        self.angle_input.setFixedWidth(60)
        motor_layout.addWidget(self.angle_input,1,1)
        self.move_button = QPushButton("Move")
        self.move_button.setEnabled(False)
        self.move_button.clicked.connect(self.on_move_motor)
        motor_layout.addWidget(self.move_button,1,2)
        self.motor_group = QGroupBox("Motor")
        self.motor_group.setLayout(motor_layout)

        # Filter Wheel Group
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("FilterWheel COM:"))
        self.filter_port_combo = QComboBox()
        self.filter_port_combo.setEditable(True)
        self.filter_port_combo.addItems(ports)
        self.filter_port_combo.setCurrentText("COM17")
        filter_layout.addWidget(self.filter_port_combo)
        filter_layout.addWidget(QLabel("Command:"))
        self.filter_cmd_input = QLineEdit()
        self.filter_cmd_input.setPlaceholderText("F1r, F15, F19 or ?")
        filter_layout.addWidget(self.filter_cmd_input)
        self.filter_send_button = QPushButton("Send")
        self.filter_send_button.setEnabled(False)
        self.filter_send_button.clicked.connect(self.on_send_filter)
        filter_layout.addWidget(self.filter_send_button)
        filter_layout.addWidget(QLabel("Current Pos:"))
        self.filter_pos_label = QLabel("--")
        filter_layout.addWidget(self.filter_pos_label)
        filter_layout.addStretch()
        self.filter_group = QGroupBox("Filter Wheel")
        self.filter_group.setLayout(filter_layout)

        # IMU Group
        imu_layout = QHBoxLayout()
        imu_layout.addWidget(QLabel("IMU COM:"))
        self.imu_port_combo = QComboBox()
        self.imu_port_combo.setEditable(True)
        self.imu_port_combo.addItems(ports)
        imu_layout.addWidget(self.imu_port_combo)
        imu_layout.addWidget(QLabel("Baud:"))
        self.imu_baud_combo = QComboBox()
        self.imu_baud_combo.addItems(["9600","57600","115200"])
        self.imu_baud_combo.setCurrentText("9600")
        imu_layout.addWidget(self.imu_baud_combo)
        self.imu_connect_button = QPushButton("Connect IMU")
        self.imu_connect_button.clicked.connect(self.on_connect_imu)
        imu_layout.addWidget(self.imu_connect_button)

        self.imu_data_label = QLabel("IMU data: not connected")
        self.figure = plt.figure(figsize=(4,4))
        self.ax = self.figure.add_subplot(111,projection='3d')
        self.ax.set_title("3D Orientation with Sun")
        self.ax.set_xlim([-3,3]); self.ax.set_ylim([-3,3]); self.ax.set_zlim([-3,3])
        self.canvas = FigureCanvas(self.figure)
        self.camera_label = QLabel()
        self.camera_label.setFixedHeight(500)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera = cv2.VideoCapture(0)
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_frame)
        self.camera_timer.start(30)

        self.imu_group = QGroupBox("IMU")
        imu_group_layout = QVBoxLayout()
        imu_group_layout.addLayout(imu_layout)
        imu_group_layout.addWidget(self.camera_label)
        imu_group_layout.addWidget(self.imu_data_label)
        imu_group_layout.addWidget(self.canvas)
        self.imu_group.setLayout(imu_group_layout)

        # Splitter Layout
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0,0,0,0)
        right_layout.setSpacing(10)
        right_layout.addWidget(self.motor_group)
        right_layout.addWidget(self.filter_group)
        right_layout.addWidget(self.imu_group)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.spectro_group)
        splitter.addWidget(right_container)
        splitter.setStretchFactor(0,3)
        splitter.setStretchFactor(1,2)
        main_layout.addWidget(splitter,stretch=1)

        # Data saving setup
        self.csv_dir = self.log_dir = "data"
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_file = None
        self.log_file = None
        self.continuous_saving = False
        self.save_data_timer = QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)

        # Connect filter wheel & reset to pos1
        self.filter_thread = filterwheel.FilterWheelConnectThread(
            self.filter_port_combo.currentText(), parent=self)
        self.filter_thread.result_signal.connect(self.on_filter_connect_result)
        self._running_threads.append(self.filter_thread)
        self.filter_thread.start()

        # Status indicators timer
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_indicators)
        self.status_timer.start(3000)
        self.update_status_indicators()

    def update_status_indicators(self):
        ports = {p.device for p in list_ports.comports()}
        # Motor
        m = self.motor_port_combo.currentText().strip()
        if m in ports:
            if self.motor_connected:
                color = "green" if not self.motor_comm_issue else "yellow"
            else:
                color = "red"
        else:
            color = "grey"
        self.motor_group.setTitle(f"<font color='{color}'>●</font> Motor")

        # Filter Wheel
        f = self.filter_port_combo.currentText().strip()
        if f in ports:
            if self.filterwheel_connected:
                color = "green" if not self.filterwheel_comm_issue else "yellow"
            else:
                color = "red"
        else:
            color = "grey"
        self.filter_group.setTitle(f"<font color='{color}'>●</font> Filter Wheel")

        # IMU
        i = self.imu_port_combo.currentText().strip()
        if i in ports:
            color = "green" if self.imu_connected else "red"
        else:
            color = "grey"
        self.imu_group.setTitle(f"<font color='{color}'>●</font> IMU")

    def on_filter_connect_result(self, ser, message):
        self.status_bar.showMessage(message)
        if ser and ser.is_open:
            self.filterwheel_serial = ser
            self.filterwheel_connected = True
            self.filter_send_button.setEnabled(True)
            # Reset to position 1
            self.last_filter_command = "F1r"
            th = filterwheel.FilterWheelCommandThread(
                ser, "F1r", parent=self)
            th.result_signal.connect(self.on_filter_command_result)
            self._running_threads.append(th)
            th.start()
        else:
            self.filterwheel_connected = False
            self.filter_send_button.setEnabled(False)
            self.filter_pos_label.setText("--")

    def on_send_filter(self):
        if not (self.filterwheel_serial and self.filterwheel_serial.is_open):
            self.status_bar.showMessage("Filter wheel not connected.")
            return
        cmd = self.filter_cmd_input.text().strip()
        if not cmd:
            self.status_bar.showMessage("Enter a command for filter wheel.")
            return
        self.filter_send_button.setEnabled(False)
        self.status_bar.showMessage(f"Sending '{cmd}' to filter wheel...")
        self.last_filter_command = cmd
        th = filterwheel.FilterWheelCommandThread(
            self.filterwheel_serial, cmd, parent=self)
        th.result_signal.connect(self.on_filter_command_result)
        self._running_threads.append(th)
        th.start()

    def on_filter_command_result(self, pos, message):
        self.filter_send_button.setEnabled(True)
        if pos is not None and self.last_filter_command == "?":
            self.filter_pos_label.setText(str(pos))
            self.filterwheel_comm_issue = False
        elif pos is None:
            self.filterwheel_comm_issue = bool(
                self.filterwheel_serial and self.filterwheel_serial.is_open)
        self.status_bar.showMessage(message)
        self.last_filter_command = None

    def on_connect_motor(self):
        if self.motor_connected:
            self.status_bar.showMessage("Motor is already connected.")
            return
        p = self.motor_port_combo.currentText().strip()
        if not p:
            self.status_bar.showMessage("Please select a COM port for the motor.")
            return
        self.motor_connect_button.setEnabled(False)
        self.status_bar.showMessage(f"Connecting to motor on {p}...")
        th = motor.MotorConnectThread(p, parent=self)
        th.result_signal.connect(self.on_motor_connect_result)
        self._running_threads.append(th)
        th.start()

    def on_motor_connect_result(self, ser, baud, message):
        self.motor_connect_button.setEnabled(True)
        if ser and ser.is_open:
            self.motor_serial = ser
            self.motor_connected = True
            self.move_button.setEnabled(True)
            self.motor_comm_issue = False
        else:
            self.motor_connected = False
            self.move_button.setEnabled(False)
            self.motor_comm_issue = True
        self.status_bar.showMessage(message)

    def on_move_motor(self):
        if not (self.motor_connected and self.motor_serial):
            self.status_bar.showMessage("Motor not connected.")
            return
        try:
            angle = int(self.angle_input.text().strip())
        except ValueError:
            self.status_bar.showMessage("Invalid angle input.")
            return
        ok = motor.send_move_command(self.motor_serial, angle)
        self.motor_comm_issue = not ok
        self.status_bar.showMessage(
            f"Motor moved to angle {angle}°." if ok else "No ACK from motor.")

    def on_connect_imu(self):
        if self.imu_connected:
            self.status_bar.showMessage("IMU is already connected.")
            return
        p = self.imu_port_combo.currentText().strip()
        try:
            b = int(self.imu_baud_combo.currentText())
        except ValueError:
            b = 9600
        try:
            self.imu_serial = serial.Serial(p, baudrate=b, timeout=1)
        except Exception as e:
            self.status_bar.showMessage(f"Failed to open IMU port: {e}")
            return
        self.imu_connected = True
        self.imu_stop_event = imu.start_imu_read_thread(self.imu_serial, self.latest_data)
        self.status_bar.showMessage(f"IMU connected on {p} at {b} baud.")
        self.imu_timer = QTimer(self)
        self.imu_timer.timeout.connect(self.update_imu_visualization)
        self.imu_timer.start(100)

    def update_camera_frame(self):
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bpl = ch * w
                img = QImage(frame.data, w, h, bpl, QImage.Format_RGB888)
                self.camera_label.setPixmap(QPixmap.fromImage(img))

    def update_imu_visualization(self):
        if not self.imu_connected:
            return
        roll, pitch, yaw = self.latest_data['rpy']
        temp = self.latest_data.get('temperature', 0.0)
        pres = self.latest_data.get('pressure', 0.0)
        lat = self.latest_data.get('latitude', 0.0)
        lon = self.latest_data.get('longitude', 0.0)
        self.imu_data_label.setText(
            f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°\n"
            f"Temp={temp:.2f} °C, Pressure={pres:.2f} hPa\n"
            f"Lat={lat:.6f}, Lon={lon:.6f}"
        )
        self._draw_device_orientation(roll, pitch, yaw, lat, lon)
        self.canvas.draw()

    def _draw_device_orientation(self, roll, pitch, yaw, lat, lon):
        self.ax.cla()
        self.ax.set_title("3D Orientation with Sun")
        self.ax.set_xlabel("X"); self.ax.set_ylabel("Y"); self.ax.set_zlabel("Z")
        self.ax.set_xlim([-3, 3]); self.ax.set_ylim([-3, 3]); self.ax.set_zlim([-3, 3])
        s = 0.2
        cube = np.array([[-s, -s, -s], [ s, -s, -s], [ s,  s, -s], [-s,  s, -s],
                         [-s, -s,  s], [ s, -s,  s], [ s,  s,  s], [-s,  s,  s]])
        rx = np.array([[1, 0, 0],
                       [0, math.cos(math.radians(roll)), -math.sin(math.radians(roll))],
                       [0, math.sin(math.radians(roll)),  math.cos(math.radians(roll))]])
        ry = np.array([[ math.cos(math.radians(pitch)), 0, math.sin(math.radians(pitch))],
                       [ 0, 1, 0],
                       [-math.sin(math.radians(pitch)), 0, math.cos(math.radians(pitch))]])
        rz = np.array([[math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)), 0],
                       [math.sin(math.radians(yaw)),  math.cos(math.radians(yaw)), 0],
                       [0, 0, 1]])
        rc = (rz @ (ry @ (rx @ cube.T))).T
        edges = [
            [rc[0], rc[1], rc[2], rc[3]],
            [rc[4], rc[5], rc[6], rc[7]],
            [rc[0], rc[1], rc[5], rc[4]],
            [rc[2], rc[3], rc[7], rc[6]],
            [rc[1], rc[2], rc[6], rc[5]],
            [rc[4], rc[7], rc[3], rc[0]],
        ]
        for e in edges:
            self.ax.add_collection3d(Poly3DCollection(
                [e], facecolors='#8f8f8f', linewidths=1, edgecolors='k', alpha=0.2))
        sx, sy, sz = utils.compute_sun_vector(lat, lon)
        self.ax.scatter([sx], [sy], [sz], marker='o', s=50)

    def on_connect_spectrometer(self):
        self.status_bar.showMessage("Connecting to spectrometer...")
        try:
            spec_handle, wavelengths, num_pixels, serial_str = spectrometer.connect_spectrometer()
        except Exception as e:
            self.status_bar.showMessage(str(e))
            return
        self.spec_handle = spec_handle
        self.wavelengths = (wavelengths.tolist()
                            if isinstance(wavelengths, np.ndarray) else wavelengths)
        self.num_pixels = num_pixels
        self.start_meas_button.setEnabled(True)
        self.save_data_button.setEnabled(False)
        self.stop_meas_button.setEnabled(False)
        self.status_bar.showMessage(
            f"Spectrometer connected (Serial: {serial_str}). Ready to measure.")

    def on_start_measurement(self):
        if not self.spec_handle or self.spec_handle == spectrometer.INVALID_AVS_HANDLE_VALUE:
            self.status_bar.showMessage("Spectrometer not connected.")
            return
        if self.measurement_active:
            return
        integration_time_ms = 50.0
        res = spectrometer.prepare_measurement(
            self.spec_handle, self.num_pixels,
            integration_time_ms=integration_time_ms, averages=1)
        if res != 0:
            self.status_bar.showMessage(f"Error preparing measurement (code {res}).")
            return
        self.measurement_active = True
        try:
            self.cb_func = spectrometer.AVS_MeasureCallbackFunc(self._spectro_callback)
            res = spectrometer.AVS_MeasureCallback(self.spec_handle, self.cb_func, -1)
        except Exception as e:
            self.status_bar.showMessage(f"Failed to start measurement: {e}")
            self.measurement_active = False
            return
        if res != 0:
            self.status_bar.showMessage(f"AVS_MeasureCallback failed (code {res}).")
            self.measurement_active = False
            return
        self.current_integration_time_us = int(integration_time_ms * 1000)
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(True)
        self.spec_timer = QTimer(self)
        self.spec_timer.timeout.connect(self.update_spectrometer_plot)
        self.spec_timer.start(200)
        self.status_bar.showMessage("Spectrometer measurement started.")

    def _spectro_callback(self, p_data, p_user):
        """Callback for AvaSpec data readiness."""
        if not p_data or not p_user:
            return
        dev_handle = p_data[0]
        status_code = p_user[0]
        if dev_handle != self.spec_handle:
            return
        if status_code == 0:
            result = spectrometer.AVS_GetScopeData(self.spec_handle)
            if result:
                timestamp, spectral_data = result
            else:
                spectral_data = None
            if spectral_data:
                intensities_full = [0.0] * self.num_pixels
                count = len(spectral_data)
                intensities_full[0:count] = spectral_data
                self.intensities = intensities_full
                self.save_data_button.setEnabled(True)
                self.toggle_save_button.setEnabled(True)
        else:
            self.status_bar.showMessage(f"Spectrometer error (code {status_code}).")

    def update_spectrometer_plot(self):
        if not self.measurement_active or not hasattr(self, 'intensities'):
            return
        self.spec_curve.setData(self.wavelengths, self.intensities)

    def on_stop_measurement(self):
        if not self.measurement_active:
            return
        self.stop_meas_button.setEnabled(False)
        if hasattr(self, 'spec_timer'):
            self.spec_timer.stop()

        stopper = spectrometer.StopMeasureThread(self.spec_handle, parent=self)
        stopper.finished_signal.connect(self.on_stop_measurement_finished)
        stopper.finished_signal.connect(lambda: self._running_threads.remove(stopper))
        stopper.start()
        self._running_threads.append(stopper)

    def on_stop_measurement_finished(self):
        self.measurement_active = False
        self.start_meas_button.setEnabled(True)
        self.save_data_button.setEnabled(True)
        self.stop_meas_button.setEnabled(False)

    def on_save_data(self):
        if not hasattr(self, 'intensities') or not self.intensities:
            self.status_bar.showMessage("No spectral data available to save.")
            return
        data_dir = "data"
        os.makedirs(data_dir, exist_ok=True)
        timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
        filename = os.path.join(data_dir, f"snapshot_{timestamp}.csv")
        try:
            with open(filename, 'w', newline='') as f:
                f.write("Wavelength (nm),Intensity\n")
                for wl, inten in zip(self.wavelengths, self.intensities):
                    if inten != 0:
                        f.write(f"{wl:.4f},{inten:.4f}\n")
            self.status_bar.showMessage(f"Snapshot saved to {filename}")
        except Exception as e:
            self.status_bar.showMessage(f"Snapshot save failed: {e}")

    def toggle_data_saving(self):
        if not self.continuous_saving:
            # Start logging
            if self.csv_file:
                self.csv_file.close()
            if self.log_file:
                self.log_file.close()
            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file = open(os.path.join(self.csv_dir, f"log_{timestamp}.csv"), 'w', newline='')
            self.log_file = open(os.path.join(self.log_dir, f"log_{timestamp}.txt"), 'w')
            # Write headers...
            self.save_data_timer.start(1000)
            self.continuous_saving = True
            self.toggle_save_button.setText("Pause Saving")
            self.status_bar.showMessage("Started continuous data logging.")
        else:
            # Stop logging
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file:
                self.csv_file.close()
            if self.log_file:
                self.log_file.close()
            self.toggle_save_button.setText("Start Saving")
            self.status_bar.showMessage("Stopped continuous data logging.")

    def save_continuous_data(self):
        if not self.csv_file or not self.log_file:
            return
        # Compose and write one data row (omitted for brevity)...

    def on_send_temp(self):
        try:
            temp = float(self.set_temp_input.text().strip())
            self.tc.set_setpoint(temp)
            self.status_bar.showMessage(f"Setpoint updated to {temp:.2f} °C")
        except Exception as e:
            self.status_bar.showMessage(f"Failed to set temperature: {e}")

    def update_temperature(self):
        try:
            t = self.tc.get_temperature()
            self.current_temp_label.setText(f"{t:.2f} °C")
        except Exception as e:
            self.current_temp_label.setText("-- °C")
            self.status_bar.showMessage(f"Error reading temperature: {e}")

    def closeEvent(self, event):
        # Stop all threads
        for th in list(self._running_threads):
            th.quit()
            th.wait()
        # Close serials, stop timers, release camera...
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    # Splash screen
    splash_pix = QPixmap("asset/splash.jpg")
    if not splash_pix.isNull():
        splash = QSplashScreen(splash_pix)
        splash.show()
        app.processEvents()
    win = MainWindow()
    win.showFullScreen()
    if 'splash' in locals():
        splash.finish(win)
    sys.exit(app.exec_())
