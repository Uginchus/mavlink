import time
from pymavlink import mavutil

# Connect to the vehicle
print("Connecting to vehicle...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the heartbeat message to find the system ID
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Define the camera component ID
camera_component_id = 100  # MAV_COMP_ID_CAMERA

# Request camera information
print("Requesting camera information...")
master.mav.command_long_send(
    master.target_system,
    camera_component_id,
    mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION,
    0,  # Confirmation
    1,  # Request
    0, 0, 0, 0, 0, 0
)

# Wait for the camera information message
msg = master.recv_match(type='CAMERA_INFORMATION', blocking=True, timeout=10)
if msg:
    print("Camera Information:", msg)
else:
    print("No response for camera information")

# Request camera settings
print("Requesting camera settings...")
master.mav.command_long_send(
    master.target_system,
    camera_component_id,
    mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS,
    0,  # Confirmation
    1,  # Request
    0, 0, 0, 0, 0, 0
)

# Wait for the camera settings message
msg = master.recv_match(type='CAMERA_SETTINGS', blocking=True, timeout=10)
if msg:
    print("Camera Settings:", msg)
else:
    print("No response for camera settings")

# Start image capture
print("Starting image capture...")
master.mav.command_long_send(
    master.target_system,
    camera_component_id,
    mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
    0,  # Confirmation
    0,  # Reserved (set to 0)
    0,  # Interval (set to 0 for single capture)
    1,  # Total number of images to capture
    1,  # Sequence number
    0, 0, 0
)

# Wait for the capture message
msg = master.recv_match(type='CAMERA_IMAGE_CAPTURED', blocking=True, timeout=10)
if msg:
    print("Image Captured:", msg)
else:
    print("No response for image capture")

# Request storage information
print("Requesting storage information...")
master.mav.command_long_send(
    master.target_system,
    camera_component_id,
    mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION,
    0,  # Confirmation
    0,  # Storage ID (0 for all)
    0, 0, 0, 0, 0, 0
)

# Wait for the storage information message
msg = master.recv_match(type='STORAGE_INFORMATION', blocking=True, timeout=10)
if msg:
    print("Storage Information:", msg)
else:
    print("No response for storage information")

# Request camera capture status
print("Requesting camera capture status...")
master.mav.command_long_send(
    master.target_system,
    camera_component_id,
    mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
    0,  # Confirmation
    1,  # Request
    0, 0, 0, 0, 0, 0
)

# Wait for the camera capture status message
msg = master.recv_match(type='CAMERA_CAPTURE_STATUS', blocking=True, timeout=10)
if msg:
    print("Camera Capture Status:", msg)
else:
    print("No response for camera capture status")