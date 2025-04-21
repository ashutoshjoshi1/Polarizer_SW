import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

# Import spectrometer library and types from Avantes AvaSpec SDK
try:
    from avaspec import *
except ImportError:
    # Dummy definitions if avaspec is not available (for syntax completeness in environments without the SDK)
    AVS_Init = lambda x: -1
    AVS_UpdateUSBDevices = lambda: 0
    AVS_GetList = lambda count: []
    AvsIdentityType = type('AvsIdentityType', (), {})
    AVS_Activate = lambda x: -1
    INVALID_AVS_HANDLE_VALUE = -1
    AVS_GetParameter = lambda handle, code: None
    AVS_GetLambda = lambda handle: []
    MeasConfigType = type('MeasConfigType', (), {})
    AVS_PrepareMeasure = lambda handle, cfg: -1
    AVS_MeasureCallbackFunc = lambda func: None
    AVS_MeasureCallback = lambda handle, cb, num: -1
    AVS_GetScopeData = lambda handle: (None, [])
    AVS_StopMeasure = lambda handle: None
    AVS_Done = lambda: None
    ERR_ETHCONN_REUSE = -2

class StopMeasureThread(QThread):
    """Thread to stop spectrometer measurement without freezing the UI."""
    finished_signal = pyqtSignal()
    def __init__(self, spec_handle, parent=None):
        super().__init__(parent)
        self.spec_handle = spec_handle
    def run(self):
        # Stop measurement (blocking call)
        AVS_StopMeasure(self.spec_handle)
        self.finished_signal.emit()

def connect_spectrometer():
    """Initialize the spectrometer and connect to the first available device. Returns (handle, wavelengths, num_pixels, serial)."""
    try:
        ret = AVS_Init(0)
    except Exception as e:
        raise Exception(f"Spectrometer initialization failed: {e}")
    if ret <= 0:
        # ret == 0: no devices; ret < 0: error
        if ret == 0:
            AVS_Done()
            raise Exception("No spectrometer found.")
        elif 'ERR_ETHCONN_REUSE' in globals() and ret == ERR_ETHCONN_REUSE:
            AVS_Done()
            raise Exception("Spectrometer in use by another instance!")
        else:
            AVS_Done()
            raise Exception(f"AVS_Init error (code {ret}).")
    # Update device list and get number of devices
    dev_count = AVS_UpdateUSBDevices()
    if dev_count < 1:
        AVS_Done()
        raise Exception("No spectrometer found.")
    # Retrieve list of devices
    id_list = AVS_GetList(dev_count)
    if not id_list:
        AVS_Done()
        raise Exception("Failed to retrieve spectrometer list.")
    # Activate the first spectrometer in the list
    dev_id = id_list[0]
    serial_str = dev_id.SerialNumber.decode().strip() if hasattr(dev_id.SerialNumber, 'decode') else str(dev_id.SerialNumber)
    avs_id = AvsIdentityType()
    avs_id.SerialNumber = dev_id.SerialNumber
    avs_id.UserFriendlyName = b"\x00"
    avs_id.Status = b'\x01'
    spec_handle = AVS_Activate(avs_id)
    if spec_handle == INVALID_AVS_HANDLE_VALUE:
        AVS_Done()
        raise Exception(f"Error opening spectrometer (Serial: {serial_str})")
    # Get spectrometer info and calibration data
    device_data = AVS_GetParameter(spec_handle, 63484)  # 63484 = device parameters
    if device_data is None:
        AVS_Done()
        raise Exception("Failed to get spectrometer parameters.")
    num_pixels = device_data.m_Detector_m_NrPixels
    start_pixel = getattr(device_data, 'm_StandAlone_m_Meas_m_StartPixel', 0)
    stop_pixel = getattr(device_data, 'm_StandAlone_m_Meas_m_StopPixel', num_pixels - 1)
    if start_pixel < 0:
        start_pixel = 0
    if stop_pixel <= start_pixel or stop_pixel > num_pixels - 1:
        stop_pixel = num_pixels - 1
    # Get wavelength calibration array
    wavelengths = AVS_GetLambda(spec_handle)
    if wavelengths:
        wavelengths = np.ctypeslib.as_array(wavelengths)
    else:
        # Fallback to pixel indices if calibration not available
        wavelengths = list(range(num_pixels))
    return spec_handle, wavelengths, num_pixels, serial_str

def prepare_measurement(spec_handle, num_pixels, integration_time_ms=50.0, averages=1):
    """Prepare the spectrometer measurement with given parameters. Returns 0 if success or an error code."""
    meas_cfg = MeasConfigType()
    # Configure measurement parameters
    meas_cfg.m_StartPixel = 0
    meas_cfg.m_StopPixel = num_pixels - 1
    meas_cfg.m_IntegrationTime = float(integration_time_ms)
    meas_cfg.m_IntegrationDelay = 0
    meas_cfg.m_NrAverages = averages
    meas_cfg.m_CorDynDark_m_Enable = 0
    meas_cfg.m_CorDynDark_m_ForgetPercentage = 0
    meas_cfg.m_Smoothing_m_SmoothPix = 0
    meas_cfg.m_Smoothing_m_SmoothModel = 0
    meas_cfg.m_SaturationDetection = 0
    meas_cfg.m_Trigger_m_Mode = 0       # software trigger
    meas_cfg.m_Trigger_m_Source = 0
    meas_cfg.m_Trigger_m_SourceType = 0
    meas_cfg.m_Control_m_StrobeControl = 0
    meas_cfg.m_Control_m_LaserDelay = 0
    meas_cfg.m_Control_m_LaserWidth = 0
    meas_cfg.m_Control_m_LaserWaveLength = 0.0
    meas_cfg.m_Control_m_StoreToRam = 0
    res = AVS_PrepareMeasure(spec_handle, meas_cfg)
    return res

def start_measurement(spec_handle, callback_func, num_scans=-1):
    """Start continuous measurement using the given callback function. Returns 0 if success or an error code."""
    cb_ptr = AVS_MeasureCallbackFunc(callback_func)
    res = AVS_MeasureCallback(spec_handle, cb_ptr, num_scans)
    return res

def stop_measurement(spec_handle):
    """Stop the ongoing spectrometer measurement (blocking call)."""
    AVS_StopMeasure(spec_handle)

def close_spectrometer():
    """Release spectrometer resources (deactivate device and cleanup)."""
    AVS_Done()
