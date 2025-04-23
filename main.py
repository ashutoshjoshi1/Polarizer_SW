import sys
import os
import math
import cv2
import serial
import numpy as np
from datetime import datetime, timezone
from serial.tools import list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout
from PyQt5.QtWidgets import QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSplitter, QStatusBar
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
            self.tc = TC36_25()  # default port COM16
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

        # --- Filter Wheel controls (added in MainWindow.__init__) ---
        filter_layout = QHBoxLayout()
        filter_layout.setSpacing(10)
        filter_layout.addWidget(QLabel("FilterWheel COM:"))
        self.filter_port_combo = QComboBox()
        self.filter_port_combo.setEditable(True)
        # Populate available ports, default to COM17
        for port in list_ports.comports():
            name = getattr(port, 'name', port.device)
            self.filter_port_combo.addItem(name)
        if self.filter_port_combo.count() == 0:
            # If no ports detected, provide some common options
            for n in range(1, 10):
                self.filter_port_combo.addItem(f"COM{n}")
        self.filter_port_combo.setCurrentText("COM17")  # default selection
        filter_layout.addWidget(self.filter_port_combo)
        
        filter_layout.addWidget(QLabel("Command:"))
        self.filter_cmd_input = QLineEdit()
        self.filter_cmd_input.setPlaceholderText("Enter filter command")
        filter_layout.addWidget(self.filter_cmd_input)
        
        self.filter_send_button = QPushButton("Send")
        self.filter_send_button.clicked.connect(self.on_send_filter)
        filter_layout.addWidget(self.filter_send_button)
        
        filter_layout.addWidget(QLabel("Current Pos:"))
        self.filter_pos_label = QLabel("--")
        filter_layout.addWidget(self.filter_pos_label)
        filter_layout.addStretch(1)  # push controls to the left if there's extra space
        
        # Group box for filter wheel
        self.filter_group = QGroupBox("Filter Wheel")
        self.filter_group.setLayout(filter_layout)
        
        # Add the Filter Wheel group box into the main layout (e.g., on the right side panel)
        right_layout.addWidget(self.motor_group)
        right_layout.addWidget(self.filter_group)       # insert Filter Wheel panel
        right_layout.addWidget(self.imu_group)
        
        # ... (other initialization code) ...
        
        # State variables for filter wheel
        self.filterwheel_serial = None
        self.filterwheel_connected = False
        self.current_filter_position = None
        self.filterwheel_port = None
        
        self.filter_send_button.setEnabled(False)  # disable Send until initial reset completes
        
        # Attempt to connect to filter wheel on startup and reset to position 1
        self.filter_thread = filterwheel.FilterWheelConnectThread("COM17")
        self.filter_thread.result_signal.connect(self.on_filter_connect_result)
        self.filter_thread.start()


        # --- IMU controls ---
        imu_controls_layout = QHBoxLayout()
        imu_com_label = QLabel("IMU COM:")
        baud_label = QLabel("Baud:")
        self.imu_port_combo = QComboBox()
        self.imu_port_combo.setEditable(True)
        for port in ports:
            name = getattr(port, 'name', port.device)
            self.imu_port_combo.addItem(name)
        for n in range(1, 10):
            pass  # placeholder if none
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

        # State variables
        self.motor_serial = None
        self.motor_connected = False
        self.spec_handle = None
        self.measurement_active = False
        self.wavelengths = []
        self.intensities = []
        self.num_pixels = 0
        self.current_motor_angle = 0.0
        self.imu_serial = None
        self.imu_connected = False
        self.latest_data = {"accel": (0.0, 0.0, 0.0), "gyro": (0.0,0.0,0.0), "mag":(0.0,0.0,0.0), "rpy":(0.0,0.0,0.0), "pressure":0.0, "temperature":0.0, "latitude":39.0, "longitude":-76.0, "TempController_curr": 20, "TempController_set": 20}

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

    def on_filter_connect_result(self, ser_obj, message):
        """Slot for filter wheel connection attempt result."""
        self.filter_send_button.setEnabled(True)  # re-enable send on completion
        if ser_obj and ser_obj.is_open:
            # Connection succeeded
            self.filterwheel_serial = ser_obj
            self.filterwheel_connected = True
            self.filterwheel_port = ser_obj.port  # store the port name
            # Immediately send "F1r" to reset wheel to position 1 in the background
            self.status_bar.showMessage("Filter wheel connected. Resetting to position 1...")
            self.filter_cmd_thread = filterwheel.FilterWheelCommandThread(self.filterwheel_serial, "F1r")
            self.filter_cmd_thread.result_signal.connect(self.on_filter_command_result)
            self.filter_cmd_thread.start()
            self.filter_send_button.setEnabled(False)  # disable until reset command finishes
        else:
            # Connection failed
            self.filterwheel_serial = None
            self.filterwheel_connected = False
            self.status_bar.showMessage(message)
            # (Leave position label as "--")
    
    def on_send_filter(self):
        """Send a user-entered command to the filter wheel."""
        cmd = self.filter_cmd_input.text().strip()
        if not cmd:
            self.status_bar.showMessage("Please enter a filter wheel command.")
            return
        port_name = self.filter_port_combo.currentText().strip()
        if not port_name:
            self.status_bar.showMessage("Please select a COM port for the filter wheel.")
            return
        # If not connected or if port changed, (re)open the serial port
        if self.filterwheel_serial is None or not self.filterwheel_serial.is_open or port_name != self.filterwheel_port:
            # Close existing connection if open
            if self.filterwheel_serial and self.filterwheel_serial.is_open:
                try: 
                    self.filterwheel_serial.close()
                except: 
                    pass
            try:
                # Open the new port (4800 baud, 8N1)
                self.filterwheel_serial = serial.Serial(port_name, baudrate=4800, timeout=1)
                self.filterwheel_port = port_name
                self.filterwheel_connected = True
            except Exception as e:
                self.filterwheel_serial = None
                self.filterwheel_connected = False
                self.status_bar.showMessage(f"Failed to open {port_name}: {e}")
                return
        # Send the command in background thread
        self.filter_cmd_thread = filterwheel.FilterWheelCommandThread(self.filterwheel_serial, cmd)
        self.filter_cmd_thread.result_signal.connect(self.on_filter_command_result)
        self.filter_cmd_thread.start()
        self.filter_send_button.setEnabled(False)  # disable button while command is in progress
    
    def on_filter_command_result(self, pos, message):
        """Slot to handle the result of a filter wheel command (position update or error)."""
        self.filter_send_button.setEnabled(True)  # re-enable the Send button now that operation finished
        if pos is not None:
            # Successfully got a position reading
            self.current_filter_position = pos
            self.filter_pos_label.setText(str(pos))
        else:
            # Communication error or no response; if serial port got closed due to error, update state
            if self.filterwheel_serial and not self.filterwheel_serial.is_open:
                self.filterwheel_serial = None
                self.filterwheel_connected = False
                self.filter_pos_label.setText("--")
        # Show the status message (e.g., "Filter wheel moved to position X." or error notice)
        self.status_bar.showMessage(message)


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
        """Start or stop continuous data logging to CSV and text log files."""
        if not self.continuous_saving:
            # Start logging to new files
            if self.csv_file or self.log_file:
                if self.csv_file: self.csv_file.close()
                if self.log_file: self.log_file.close()
            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file_path = os.path.join(self.csv_dir, f"log_{timestamp}.csv")
            self.log_file_path = os.path.join(self.log_dir, f"log_{timestamp}.txt")
            try:
                self.csv_file = open(self.csv_file_path, 'w', newline='', encoding='utf-8')
                self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
                # Write CSV header
                header_fields = [
                    "Timestamp", "MotorPos_steps", "MotorSpeed_steps_s", "MotorCurrent_pct",
                    "MotorAlarmCode", "MotorTemp_C", "MotorAngle_deg", "FilterWheel 1", 
                    "Roll_deg", "Pitch_deg", "Yaw_deg",
                    "AccelX_g", "AccelY_g", "AccelZ_g", "GyroX_dps", "GyroY_dps", "GyroZ_dps",
                    "MagX_uT", "MagY_uT", "MagZ_uT", "Pressure_hPa", "Temperature_C",
                    "Latitude_deg", "Longitude_deg", "IntegrationTime_us", "TempController_curr", 
                    "TempController_set"
                ]
                # Append one column per wavelength (e.g., I_500.00nm)
                for wl in (self.wavelengths if isinstance(self.wavelengths, (list, np.ndarray)) else []):
                    header_fields.append(f"I_{float(wl):.2f}nm")
                self.csv_file.write(",".join(header_fields) + "\n")
                # Start periodic logging (1 Hz)
                self.save_data_timer.start(1000)
                self.continuous_saving = True
                self.toggle_save_button.setText("Pause Saving")
                self.status_bar.showMessage(f"Started saving data to {self.csv_file_path}")
            except Exception as e:
                self.status_bar.showMessage(f"Failed to start saving: {e}")
                # Ensure no file is left open on failure
                if self.csv_file:
                    self.csv_file.close()
                    self.csv_file = None
                if self.log_file:
                    self.log_file.close()
                    self.log_file = None
                return
        else:
            # Stop logging
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.toggle_save_button.setText("Start Saving")
            if self.csv_file_path and self.log_file_path:
                self.status_bar.showMessage(f"Data saved to {self.csv_file_path} and {self.log_file_path}")
            else:
                self.status_bar.showMessage("Data logging stopped.")
    def save_continuous_data(self):
        """Log one data sample (sensors + spectrum) to the CSV and text files."""
        if not self.csv_file or not self.log_file:
            return  # Not actively logging
        now = QDateTime.currentDateTime()
        ts_csv = now.toString("yyyy-MM-dd hh:mm:ss.zzz")
        ts_txt = now.toString("yyyy-MM-dd hh:mm:ss")
        # Motor data
        motor_pos = int(self.current_motor_angle)
        motor_angle = float(self.current_motor_angle)
        motor_speed = motor.TrackerSpeed
        motor_current_pct = motor.TrackerCurrent / 10.0  # e.g., 1000 -> 100.0%
        motor_alarm = 0
        motor_temp = None  # (motor temperature not available)
        # IMU data
        roll, pitch, yaw = self.latest_data["rpy"]
        ax_g, ay_g, az_g = self.latest_data.get("accel", (0.0, 0.0, 0.0))
        gx_dps, gy_dps, gz_dps = self.latest_data.get("gyro", (0.0, 0.0, 0.0))
        mx_uT, my_uT, mz_uT = self.latest_data.get("mag", (0.0, 0.0, 0.0))
        pres = self.latest_data.get("pressure", 0.0)
        temp = self.latest_data.get("temperature", 0.0)
        lat = self.latest_data.get("latitude", 0.0)
        lon = self.latest_data.get("longitude", 0.0)
        integration_us = getattr(self, "current_integration_time_us", 0)
        TempController_curr = self.latest_data.get("TempController_curr", 0.0)
        TempController_set = self.latest_data.get("TempController_set", 0.0)
        # Prepare CSV row fields
        row_fields = [
            ts_csv,
            str(motor_pos),
            str(motor_speed),
            f"{motor_current_pct:.1f}",
            str(motor_alarm),
            "" if motor_temp is None else f"{motor_temp:.1f}",
            f"{motor_angle:.1f}",
            (str(self.current_filter_position) if self.current_filter_position is not None else ""), 
            f"{roll:.2f}",
            f"{pitch:.2f}",
            f"{yaw:.2f}",
            f"{ax_g:.2f}",
            f"{ay_g:.2f}",
            f"{az_g:.2f}",
            f"{gx_dps:.2f}",
            f"{gy_dps:.2f}",
            f"{gz_dps:.2f}",
            f"{mx_uT:.2f}",
            f"{my_uT:.2f}",
            f"{mz_uT:.2f}",
            f"{pres:.2f}",
            f"{temp:.2f}",
            f"{lat:.6f}",
            f"{lon:.6f}",
            str(int(integration_us)),
            f"{TempController_curr:2f}",
            f"{TempController_set:2f}"
        ]
        # Append intensity values for each wavelength (empty string if intensity is zero or missing)
        if self.intensities and isinstance(self.wavelengths, (list, np.ndarray)) and len(self.intensities) == len(self.wavelengths):
            for inten in self.intensities:
                if inten != 0:
                    row_fields.append(f"{inten:.4f}")
                else:
                    row_fields.append("")
        else:
            for _ in (self.wavelengths if isinstance(self.wavelengths, (list, np.ndarray)) else []):
                row_fields.append("")
        # Write CSV line
        csv_line = ",".join(row_fields)
        try:
            self.csv_file.write(csv_line + "\n")
        except Exception as e:
            self.status_bar.showMessage(f"Error during data save: {e}")
        # Write text log line (human-readable summary)
        peak_text = ""
        if self.intensities and len(self.intensities) > 0:
            max_intensity = max(self.intensities)
            if max_intensity != 0:
                max_idx = self.intensities.index(max_intensity) if isinstance(self.intensities, list) else int(np.argmax(self.intensities))
                peak_text = f"Peak {max_intensity:.1f} at {self.wavelengths[max_idx]:.1f} nm"
            else:
                peak_text = "Peak 0 at N/A nm"
        else:
            peak_text = "Peak 0 at N/A nm"
        motor_temp_text = f"{motor_temp:.1f}°C" if motor_temp is not None else "N/A"
        log_line = (f"Time {ts_txt}: Motor at {motor_pos} steps ({motor_angle:.0f}°), "
                    f"Speed {motor_speed} steps/s, Current {motor_current_pct:.1f}%, "
                    f"Temp {motor_temp_text}, AlarmCode {motor_alarm}; "
                    f"Spectrometer {peak_text}; IMU Orientation: Roll {roll:.1f}°, "
                    f"Pitch {pitch:.1f}°, Yaw {yaw:.1f}°; GPS: Lat {lat:.6f}°, Lon {lon:.6f}°.\n")
        try:
            self.log_file.write(log_line)
        except Exception as e:
            self.status_bar.showMessage(f"Error writing log: {e}")

    # --- Temperature control handlers ---
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
