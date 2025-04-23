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
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSplitter, QStatusBar
)
from PyQt5.QtCore import QTimer, Qt, QDateTime
from PyQt5.QtGui import QPixmap, QImage
import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import motor
import imu
import spectrometer
import utils
from temp_controller import TC36_25

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1000, 750)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(8)

        # Prepare data logging state and directory
        self.continuous_saving = False
        self.save_data_timer = QTimer()
        self.save_data_timer.timeout.connect(self.save_continuous_data)
        self.csv_dir = "data_logs/csv"
        self.log_dir = "data_logs/logs"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_file = None
        self.log_file = None
        self.csv_file_path = None
        self.log_file_path = None

        # Logo (optional)
        logo_layout = QHBoxLayout()
        logo_layout.addStretch()
        logo_label = QLabel()
        pixmap = QPixmap("asset/sciglob_symbol.png")
        if not pixmap.isNull():
            logo_label.setPixmap(
                pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        logo_layout.addWidget(logo_label)
        main_layout.addLayout(logo_layout)

        # --- Temperature Control section ---
        temp_layout = QHBoxLayout()
        temp_layout.setSpacing(10)
        temp_layout.addWidget(QLabel("Set Temp (°C):"))
        self.set_temp_input = QLineEdit()
        self.set_temp_input.setFixedWidth(60)
        self.set_temp_input.setText("20.0")  # default 20°C
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
            self.tc = TC36_25()
            self.tc.enable_computer_setpoint()
            self.tc.power(True)
            # Timer to update current temperature every second
            self.temp_timer = QTimer()
            self.temp_timer.timeout.connect(self.update_temperature)
            self.temp_timer.start(1000)
        except Exception as e:
            self.status_bar.showMessage(f"Temp controller init failed: {e}")

        # --- Spectrometer controls ---
        spectro_controls_layout = QHBoxLayout()
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.setEnabled(False)
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        self.connect_spec_button = QPushButton("Connect Spectrometer")
        self.connect_spec_button.clicked.connect(self.on_connect_spectrometer)
        self.start_meas_button = QPushButton("Start Measurement")
        self.start_meas_button.clicked.connect(self.on_start_measurement)
        self.stop_meas_button = QPushButton("Stop")
        self.stop_meas_button.clicked.connect(self.on_stop_measurement)
        self.save_data_button = QPushButton("Save Data")
        self.save_data_button.clicked.connect(self.on_save_data)
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(False)
        self.save_data_button.setEnabled(False)
        spectro_controls_layout.addWidget(self.toggle_save_button)
        spectro_controls_layout.addWidget(self.connect_spec_button)
        spectro_controls_layout.addWidget(self.start_meas_button)
        spectro_controls_layout.addWidget(self.stop_meas_button)
        spectro_controls_layout.addWidget(self.save_data_button)

        # --- Motor controls ---
        motor_layout = QGridLayout()
        motor_com_label = QLabel("Motor COM:")
        angle_label = QLabel("Angle (°):")
        self.motor_port_combo = QComboBox()
        self.motor_port_combo.setEditable(True)
        ports = list_ports.comports()
        for port in ports:
            name = getattr(port, 'name', port.device)
            self.motor_port_combo.addItem(name)
        if self.motor_port_combo.count() == 0:
            for n in range(1, 10):
                self.motor_port_combo.addItem(f"COM{n}")
        self.motor_connect_button = QPushButton("Connect Motor")
        self.motor_connect_button.clicked.connect(self.on_connect_motor)
        self.angle_input = QLineEdit()
        self.angle_input.setFixedWidth(60)
        self.move_button = QPushButton("Move")
        self.move_button.clicked.connect(self.on_move_motor)
        self.move_button.setEnabled(False)
        motor_layout.addWidget(motor_com_label, 0, 0)
        motor_layout.addWidget(self.motor_port_combo, 0, 1)
        motor_layout.addWidget(self.motor_connect_button, 0, 2)
        motor_layout.addWidget(angle_label, 1, 0)
        motor_layout.addWidget(self.angle_input, 1, 1)
        motor_layout.addWidget(self.move_button, 1, 2)

        # --- IMU controls ---
        imu_controls_layout = QHBoxLayout()
        imu_com_label = QLabel("IMU COM:")
        baud_label = QLabel("Baud:")
        self.imu_port_combo = QComboBox()
        self.imu_port_combo.setEditable(True)
        for port in ports:
            name = getattr(port, 'name', port.device)
            self.imu_port_combo.addItem(name)
        self.imu_baud_combo = QComboBox()
        for b in [9600, 57600, 115200]:
            self.imu_baud_combo.addItem(str(b))
        self.imu_baud_combo.setCurrentText("9600")
        self.imu_connect_button = QPushButton("Connect IMU")
        self.imu_connect_button.clicked.connect(self.on_connect_imu)
        imu_controls_layout.addWidget(imu_com_label)
        imu_controls_layout.addWidget(self.imu_port_combo)
        imu_controls_layout.addWidget(baud_label)
        imu_controls_layout.addWidget(self.imu_baud_combo)
        imu_controls_layout.addWidget(self.imu_connect_button)

        # --- Spectrometer plot setup ---
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.spec_plot_widget = pg.PlotWidget()
        self.spec_plot_widget.setLabel('bottom', 'Wavelength', units='nm')
        self.spec_plot_widget.setLabel('left', 'Intensity', units='counts')
        self.spec_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.spec_plot_widget.setXRange(260, 600)
        self.spec_curve = self.spec_plot_widget.plot([], [], pen=pg.mkPen('#2986cc', width=1))

        # --- IMU orientation plot and camera feed ---
        self.imu_data_label = QLabel("IMU data: not connected")
        self.figure = plt.figure(figsize=(4, 4))
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_title("3D Orientation with Sun")
        self.ax.set_xlabel("X"); self.ax.set_ylabel("Y"); self.ax.set_zlabel("Z")
        self.ax.set_xlim([-3, 3]); self.ax.set_ylim([-3, 3]); self.ax.set_zlim([-3, 3])
        self.canvas = FigureCanvas(self.figure)
        self.camera_label = QLabel()
        self.camera_label.setFixedHeight(500)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW) if hasattr(cv2, 'CAP_DSHOW') else cv2.VideoCapture(0)
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_frame)
        self.camera_timer.start(30)

        # --- Group boxes and layout ---
        self.spectro_group = QGroupBox("Spectrometer")
        spectro_group_layout = QVBoxLayout()
        spectro_group_layout.addLayout(spectro_controls_layout)
        spectro_group_layout.addWidget(self.spec_plot_widget, 1)
        self.spectro_group.setLayout(spectro_group_layout)
        self.motor_group = QGroupBox("Motor")
        self.motor_group.setLayout(motor_layout)
        self.imu_group = QGroupBox("IMU")
        imu_group_layout = QVBoxLayout()
        imu_group_layout.addLayout(imu_controls_layout)
        imu_group_layout.addWidget(self.camera_label)
        imu_group_layout.addWidget(self.imu_data_label)
        imu_group_layout.addWidget(self.canvas)
        self.imu_group.setLayout(imu_group_layout)
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        right_layout.addWidget(self.motor_group)
        right_layout.addWidget(self.imu_group)
        plots_splitter = QSplitter(Qt.Horizontal)
        plots_splitter.addWidget(self.spectro_group)
        plots_splitter.addWidget(right_container)
        plots_splitter.setStretchFactor(0, 3)
        plots_splitter.setStretchFactor(1, 2)
        main_layout.addWidget(plots_splitter, stretch=1)

        # Latest sensor data
        self.latest_data = {
            "accel": (0.0, 0.0, 0.0),
            "gyro": (0.0, 0.0, 0.0),
            "mag": (0.0, 0.0, 0.0),
            "rpy": (0.0, 0.0, 0.0),
            "pressure": 0.0,
            "temperature": 0.0,
            "latitude": 39.0,
            "longitude": -76.0,
            "TempController_curr": 0.0,
            "TempController_set": 20.0
        }
    def update_camera_frame(self):
        # Update camera frame to the QLabel
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.camera_label.setPixmap(QPixmap.fromImage(qt_img))
    def on_connect_motor(self):
        """Connect to the motor by opening its serial port (auto-detect baud)."""
        if self.motor_connected:
            self.status_bar.showMessage("Motor is already connected.")
            return
        port_name = self.motor_port_combo.currentText().strip()
        if not port_name:
            self.status_bar.showMessage("Please select a COM port for the motor.")
            return
        # Disable connect button during attempt
        self.motor_connect_button.setEnabled(False)
        self.status_bar.showMessage(f"Connecting to motor on {port_name}...")
        # Start background thread to connect to motor
        self.motor_thread = motor.MotorConnectThread(port_name)
        self.motor_thread.result_signal.connect(self.on_motor_connect_result)
        self.motor_thread.start()
    def on_motor_connect_result(self, ser_obj, baud, message):
        """Handle the result of the motor connection attempt."""
        self.motor_connect_button.setEnabled(True)
        if ser_obj and ser_obj.is_open:
            # Success: store serial and enable controls
            self.motor_serial = ser_obj
            self.motor_connected = True
            self.move_button.setEnabled(True)
        else:
            # Connection failed
            if self.motor_serial:
                try:
                    self.motor_serial.close()
                except:
                    pass
            self.motor_serial = None
            self.motor_connected = False
            self.move_button.setEnabled(False)
        # Display the status message (success or failure)
        self.status_bar.showMessage(message)
    def on_move_motor(self):
        """Send a move command to the motor to go to the specified angle."""
        if not self.motor_connected or self.motor_serial is None:
            self.status_bar.showMessage("Motor not connected.")
            return
        angle_text = self.angle_input.text().strip()
        try:
            angle = int(angle_text)
        except ValueError:
            self.status_bar.showMessage("Invalid angle input. Enter an integer.")
            return
        success = motor.send_move_command(self.motor_serial, angle)
        if success:
            self.current_motor_angle = angle
            self.status_bar.showMessage(f"Motor moved to angle {angle}°.")
        else:
            self.status_bar.showMessage("Motor move command sent, but no ACK received.")
    def on_connect_imu(self):
        """Connect to the IMU via serial and start the reading thread."""
        if self.imu_connected:
            self.status_bar.showMessage("IMU is already connected.")
            return
        port_name = self.imu_port_combo.currentText().strip()
        baud_str = self.imu_baud_combo.currentText().strip()
        if not port_name:
            self.status_bar.showMessage("Please select a COM port for the IMU.")
            return
        try:
            baud = int(baud_str)
        except ValueError:
            baud = 9600
        try:
            self.imu_serial = serial.Serial(port_name, baudrate=baud, timeout=1)
        except Exception as e:
            self.status_bar.showMessage(f"Failed to open IMU port: {e}")
            return
        self.imu_connected = True
        # Start background thread to continuously read IMU data
        self.imu_stop_event = imu.start_imu_read_thread(self.imu_serial, self.latest_data)
        self.status_bar.showMessage(f"IMU connected on {port_name} at {baud} baud.")
        # Start timer to update 3D visualization periodically (~10 Hz)
        self.imu_timer = QTimer()
        self.imu_timer.timeout.connect(self.update_imu_visualization)
        self.imu_timer.start(100)
    def update_imu_visualization(self):
        """Update the 3D orientation plot and IMU data label with latest readings."""
        if not self.imu_connected:
            return
        roll, pitch, yaw = self.latest_data["rpy"]
        temp = self.latest_data.get("temperature", 0.0)
        pres = self.latest_data.get("pressure", 0.0)
        lat = self.latest_data.get("latitude", 0.0)
        lon = self.latest_data.get("longitude", 0.0)
        # Update IMU data text label (roll, pitch, yaw, etc.)
        self.imu_data_label.setText(
            f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°\n"
            f"Temp={temp:.2f} °C, Pressure={pres:.2f} hPa\n"
            f"Lat={lat:.6f}, Lon={lon:.6f}"
        )
        # Redraw 3D orientation cube (and optionally sun position)
        self._draw_device_orientation(roll, pitch, yaw, lat, lon)
        self.canvas.draw()
    def _draw_device_orientation(self, roll, pitch, yaw, lat, lon):
        """Draw a cube representing the device orientation (in 3D) and optionally the sun position."""
        self.ax.cla()
        self.ax.set_title("3D Orientation with Sun")
        self.ax.set_xlabel("X"); self.ax.set_ylabel("Y"); self.ax.set_zlabel("Z")
        self.ax.set_xlim([-3, 3]); self.ax.set_ylim([-3, 3]); self.ax.set_zlim([-3, 3])
        # Define cube vertices (cube side ~0.4 units) at origin
        size = 0.2
        cube = np.array([[-size, -size, -size],
                         [ size, -size, -size],
                         [ size,  size, -size],
                         [-size,  size, -size],
                         [-size, -size,  size],
                         [ size, -size,  size],
                         [ size,  size,  size],
                         [-size,  size,  size]])
        # Rotation matrices for roll (X-axis), pitch (Y-axis), yaw (Z-axis) in Z-Y-X sequence
        rx = np.array([
            [1, 0, 0],
            [0, math.cos(math.radians(roll)), -math.sin(math.radians(roll))],
            [0, math.sin(math.radians(roll)),  math.cos(math.radians(roll))]
        ])
        ry = np.array([
            [ math.cos(math.radians(pitch)), 0, math.sin(math.radians(pitch))],
            [ 0, 1, 0],
            [-math.sin(math.radians(pitch)), 0, math.cos(math.radians(pitch))]
        ])
        rz = np.array([
            [math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)), 0],
            [math.sin(math.radians(yaw)),  math.cos(math.radians(yaw)), 0],
            [0, 0, 1]
        ])
        # Rotate cube points
        rotated_cube = (rz @ (ry @ (rx @ cube.T))).T
        # Define cube edges (pairs of vertices)
        edges = [
            [rotated_cube[0], rotated_cube[1], rotated_cube[2], rotated_cube[3]],
            [rotated_cube[4], rotated_cube[5], rotated_cube[6], rotated_cube[7]],
            [rotated_cube[0], rotated_cube[1], rotated_cube[5], rotated_cube[4]],
            [rotated_cube[2], rotated_cube[3], rotated_cube[7], rotated_cube[6]],
            [rotated_cube[1], rotated_cube[2], rotated_cube[6], rotated_cube[5]],
            [rotated_cube[4], rotated_cube[7], rotated_cube[3], rotated_cube[0]]
        ]
        for edge in edges:
            verts = [list(edge)]
            self.ax.add_collection3d(Poly3DCollection(verts, facecolors='#8f8f8f', linewidths=1, edgecolors='k', alpha=0.2))
        # (Optional) Plot sun position relative to IMU orientation
        # To add: use utils.compute_sun_vector(lat, lon) and plot a point if desired.
        # Currently left as an optional extension.
    def on_connect_spectrometer(self):
        """Connect to the spectrometer (initialize AvaSpec and activate device)."""
        self.status_bar.showMessage("Connecting to spectrometer...")
        try:
            spec_handle, wavelengths, num_pixels, serial_str = spectrometer.connect_spectrometer()
        except Exception as e:
            self.status_bar.showMessage(str(e))
            return
        # Store spectrometer handle and calibration data
        self.spec_handle = spec_handle
        # Convert wavelengths to a Python list for easier use
        self.wavelengths = wavelengths.tolist() if isinstance(wavelengths, np.ndarray) else wavelengths
        self.num_pixels = num_pixels
        # Enable measurement button now that spectrometer is connected
        self.start_meas_button.setEnabled(True)
        self.save_data_button.setEnabled(False)
        self.stop_meas_button.setEnabled(False)
        self.status_bar.showMessage(f"Spectrometer connected (Serial: {serial_str}). Ready to measure.")
    def on_start_measurement(self):
        """Configure and start continuous spectral measurement."""
        if self.spec_handle is None or self.spec_handle == spectrometer.INVALID_AVS_HANDLE_VALUE:
            self.status_bar.showMessage("Spectrometer not connected.")
            return
        if self.measurement_active:
            return  # already measuring
        # Prepare measurement (integration time, etc.)
        integration_time_ms = 50.0  # integration time in milliseconds
        res = spectrometer.prepare_measurement(self.spec_handle, self.num_pixels, integration_time_ms=integration_time_ms, averages=1)
        if res != 0:
            self.status_bar.showMessage(f"Error preparing measurement (code {res}).")
            return
        # Start measurement with callback for continuous acquisition
        self.measurement_active = True
        try:
            # Register callback function for data ready events
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
        # Record current integration time (µs) for logging
        self.current_integration_time_us = int(integration_time_ms * 1000)
        # Update UI state
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(True)
        # Start a timer to update the spectrometer plot at 5 Hz
        self.spec_timer = QTimer()
        self.spec_timer.timeout.connect(self.update_spectrometer_plot)
        self.spec_timer.start(200)
        self.status_bar.showMessage("Spectrometer measurement started.")
    def _spectro_callback(self, p_data, p_user):
        """Callback function called by AvaSpec DLL when new spectral data is ready."""
        # p_data[0] = device handle, p_user[0] = status code
        if not p_data or not p_user:
            return
        dev_handle = p_data[0]
        status_code = p_user[0]
        if dev_handle != self.spec_handle:
            return  # ignore if not for our device
        if status_code == 0:  # measurement successful
            result = spectrometer.AVS_GetScopeData(self.spec_handle)
            if result:
                timestamp, spectral_data = result
            else:
                spectral_data = None
            if spectral_data:
                # Ensure intensity array covers full pixel range
                intensities_full = [0.0] * self.num_pixels
                count = len(spectral_data)
                intensities_full[0:count] = spectral_data
                self.intensities = intensities_full
                # Enable save buttons after first data is received
                self.save_data_button.setEnabled(True)
                self.toggle_save_button.setEnabled(True)
        else:
            # Handle measurement error codes if needed
            self.status_bar.showMessage(f"Spectrometer error (code {status_code}).")
    def update_spectrometer_plot(self):
        """Update the spectrometer plot with the latest data."""
        if not self.measurement_active or not self.intensities:
            return
        self.spec_curve.setData(self.wavelengths, self.intensities)
    def on_stop_measurement(self):
        """Stop the ongoing spectrometer measurement."""
        if not self.measurement_active or self.spec_handle is None or self.spec_handle == spectrometer.INVALID_AVS_HANDLE_VALUE:
            return
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(False)
        self.status_bar.showMessage("Stopping spectrometer measurement...")
        # Use a thread to stop measurement to avoid blocking the UI thread
        self.stop_thread = spectrometer.StopMeasureThread(self.spec_handle)
        self.stop_thread.finished_signal.connect(self.on_measurement_stopped)
        self.stop_thread.start()
    def on_measurement_stopped(self):
        """Clean up after the spectrometer measurement has stopped."""
        self.measurement_active = False
        # Re-enable start button, disable stop button
        self.start_meas_button.setEnabled(True)
        self.stop_meas_button.setEnabled(False)
        self.status_bar.showMessage("Spectrometer measurement stopped.")
        # Stop plot update timer
        if hasattr(self, 'spec_timer'):
            self.spec_timer.stop()
    def on_save_data(self):
        """Save the current spectral data to a CSV file (snapshot of wavelengths and intensities)."""
        if not self.intensities or not hasattr(self.wavelengths, '__len__'):
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
            if self.csv_file: self.csv_file.close()
            if self.log_file: self.log_file.close()
            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file_path = os.path.join(self.csv_dir, f"log_{timestamp}.csv")
            self.log_file_path = os.path.join(self.log_dir, f"log_{timestamp}.txt")
            try:
                self.csv_file = open(self.csv_file_path, 'w', newline='', encoding='utf-8')
                self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
                headers = [
                    "Timestamp","MotorPos_steps","MotorSpeed_steps_s","MotorCurrent_pct",
                    "MotorAlarmCode","MotorTemp_C","MotorAngle_deg",
                    "Roll_deg","Pitch_deg","Yaw_deg",
                    "AccelX_g","AccelY_g","AccelZ_g","GyroX_dps","GyroY_dps","GyroZ_dps",
                    "MagX_uT","MagY_uT","MagZ_uT","Pressure_hPa","Temperature_C",
                    "Latitude_deg","Longitude_deg","IntegrationTime_us",
                    "TempController_curr","TempController_set"
                ]
                for wl in self.wavelengths:
                    headers.append(f"I_{wl:.2f}nm")
                self.csv_file.write(",".join(headers)+"\n")
                self.save_data_timer.start(1000)
                self.continuous_saving = True
                self.toggle_save_button.setText("Pause Saving")
                self.status_bar.showMessage(f"Started saving data to {self.csv_file_path}")
            except Exception as e:
                self.status_bar.showMessage(f"Failed to start saving: {e}")
        else:
            # Stop logging
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file: self.csv_file.close(); self.csv_file=None
            if self.log_file: self.log_file.close(); self.log_file=None
            self.toggle_save_button.setText("Start Saving")
            self.status_bar.showMessage("Data logging stopped.")

    def save_continuous_data(self):
        if not self.csv_file or not self.log_file:
            return

        # Timestamp for CSV (with ms) and for log (no ms)
        now = QDateTime.currentDateTime()
        ts_csv = now.toString("yyyy-MM-dd hh:mm:ss.zzz")
        ts_txt = now.toString("yyyy-MM-dd hh:mm:ss")

        # Motor data
        motor_pos = int(self.current_motor_angle)
        motor_speed = motor.TrackerSpeed
        motor_current_pct = motor.TrackerCurrent / 10.0
        motor_alarm = 0
        motor_temp = None  # replace with real reading if you have it

        # IMU data
        roll, pitch, yaw = self.latest_data["rpy"]
        ax, ay, az = self.latest_data["accel"]
        gx, gy, gz = self.latest_data["gyro"]
        mx, my, mz = self.latest_data["mag"]
        pres = self.latest_data["pressure"]
        temp = self.latest_data["temperature"]
        lat = self.latest_data["latitude"]
        lon = self.latest_data["longitude"]

        # Spectrometer data
        integ = getattr(self, "current_integration_time_us", 0)
        tc_curr = self.latest_data.get("TempController_curr", 0.0)
        tc_set = self.latest_data.get("TempController_set", 0.0)

        # --- Write CSV row ---
        row = [
            ts_csv,
            str(motor_pos),
            str(motor_speed),
            f"{motor_current_pct:.1f}",
            str(motor_alarm),
            "" if motor_temp is None else f"{motor_temp:.1f}",
            f"{motor_pos:.1f}",
            f"{roll:.2f}",
            f"{pitch:.2f}",
            f"{yaw:.2f}",
            f"{ax:.2f}",
            f"{ay:.2f}",
            f"{az:.2f}",
            f"{gx:.2f}",
            f"{gy:.2f}",
            f"{gz:.2f}",
            f"{mx:.2f}",
            f"{my:.2f}",
            f"{mz:.2f}",
            f"{pres:.2f}",
            f"{temp:.2f}",
            f"{lat:.6f}",
            f"{lon:.6f}",
            str(int(integ)),
            f"{tc_curr:.2f}",
            f"{tc_set:.2f}"
        ]
        # append spectral intensities
        for inten in self.intensities:
            row.append(f"{inten:.4f}" if inten != 0 else "")

        try:
            self.csv_file.write(",".join(row) + "\n")
            self.csv_file.flush()
        except Exception as e:
            self.status_bar.showMessage(f"Error writing CSV: {e}")

        # --- Build and write human-readable log line ---
        # find peak
        if self.intensities:
            max_int = max(self.intensities)
            if max_int != 0:
                idx = self.intensities.index(max_int)
                peak_text = f"Peak {max_int:.1f} at {self.wavelengths[idx]:.1f} nm"
            else:
                peak_text = "Peak 0 at N/A nm"
        else:
            peak_text = "Peak 0 at N/A nm"

        motor_temp_text = f"{motor_temp:.1f}°C" if motor_temp is not None else "N/A"

        log_line = (
            f"Time {ts_txt}: "
            f"Motor at {motor_pos} steps ({motor_pos}°), "
            f"Speed {motor_speed} steps/s, "
            f"Current {motor_current_pct:.1f}%, "
            f"Temp {motor_temp_text}, "
            f"AlarmCode {motor_alarm}; "
            f"Spectrometer {peak_text}; "
            f"IMU: Roll {roll:.1f}°, Pitch {pitch:.1f}°, Yaw {yaw:.1f}°; "
            f"GPS: {lat:.6f},{lon:.6f}; "
            f"TempCtrl Curr {tc_curr:.2f}°C, Set {tc_set:.2f}°C\n"
        )

        try:
            self.log_file.write(log_line)
            self.log_file.flush()
        except Exception as e:
            self.status_bar.showMessage(f"Error writing log: {e}")


    # --- Temperature control handlers ---
    def on_send_temp(self):
        try:
            temp = float(self.set_temp_input.text().strip())
            self.tc.set_setpoint(temp)
            self.latest_data["TempController_set"] = temp
            self.status_bar.showMessage(f"Setpoint updated to {temp:.2f} °C")
        except Exception as e:
            self.status_bar.showMessage(f"Failed to set temperature: {e}")

    def update_temperature(self):
        try:
            t = self.tc.get_temperature()
            self.current_temp_label.setText(f"{t:.2f} °C")
            self.latest_data["TempController_curr"] = t
        except Exception as e:
            self.current_temp_label.setText("-- °C")
            self.status_bar.showMessage(f"Error reading temperature: {e}")

            
    def closeEvent(self, event):
        """Cleanup on application close."""
        # Stop spectrometer measurement if running
        if hasattr(self, 'measurement_active') and self.measurement_active and self.spec_handle is not None:
            try:
                spectrometer.stop_measurement(self.spec_handle)
            except Exception:
                pass
        # Release spectrometer resources
        try:
            spectrometer.close_spectrometer()
        except Exception:
            pass
        # Close motor serial port
        if self.motor_serial and self.motor_serial.is_open:
            try:
                self.motor_serial.close()
            except Exception:
                pass
        # Stop IMU thread and close IMU serial port
        if self.imu_connected:
            try:
                if hasattr(self, 'imu_stop_event'):
                    self.imu_stop_event.set()  # signal the IMU thread to stop
            except Exception:
                pass
            if self.imu_serial and self.imu_serial.is_open:
                try:
                    self.imu_serial.close()
                except Exception:
                    pass
        # Release the camera
        if hasattr(self, 'camera') and self.camera.isOpened():
            self.camera.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
