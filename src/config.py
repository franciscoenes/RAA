import numpy as np

# === Operation Mode ===
SIMULATION_MODE = True

# === Camera Configuration ===
CAMERA_INDEX = 0
CAMERA_PARAMS_FILE = "camera_params.npz"
CAMERA_HEIGHT_CM = 69.0  # Camera height in centimeters

# === Marker Configuration ===
MARKER_SIZE_CM = 9.5
MARKER_LENGTH_METERS = 0.097  # 9.7cm in meters
VISION_SCALE_FACTOR = 0.01

# ArUco marker IDs
ARUCO_ID_HOME = 0
ARUCO_ID_CAR = 1
ARUCO_ID_DROPOFF = 2

# Home position thresholds (in meters)
HOME_MIN_THRESHOLD = 0.21  # 21cm
HOME_MAX_THRESHOLD = 0.22  # 22cm

# === Robot Arm Configuration ===
# Initial angles (in radians)
INITIAL_ARM_ANGLES_RAD = [0.0, np.radians(45), np.radians(-45)]

# Robot dimensions
LINK_LENGTHS_METERS = [0.10, 0.15, 0.15]

# Joint limits (in degrees)
JOINT_LIMITS_DEG = {
    'base': [-90, 90],
    'shoulder': [-90, 90],
    'elbow': [-90, 90]
}

# === Workspace Limits (in centimeters) ===
WORKSPACE_LIMITS = {
    'xy_min': 15.0,
    'xy_max': 28.0,
    'z_min': 5.0,
    'z_max': 25.0
}

# === Home Position (in centimeters) ===
HOME_POSITION = np.array([22.0, 0.0, 18.0])

# === Arduino Configuration ===
ARDUINO_PORT = 'COM9'
ARDUINO_BAUD_RATE = 57600

# === Trajectory Planning ===
TRAJECTORY_DT = 0.05  # Time step for trajectory planning

# === Simulation Display ===
SIMULATION_PLOT_LIMITS = {
    'x': (-0.2, 0.5),
    'y': (-0.3, 0.3),
    'z': (0, 0.4)
}

# === Status Colors (BGR format) ===
STATUS_COLORS = {
    'success': (0, 255, 0),    # Green
    'error': (0, 0, 255),      # Red
    'neutral': (255, 255, 255)  # White
} 