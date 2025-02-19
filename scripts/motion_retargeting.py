## NOTE: THIS CODE IS RUNNING BUT THE RETARGETED MOTION IS NOT GOOOD.
# Motion retargeting might not work perfectly, needs debugging & adjustments!

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
import mujoco.viewer
import time
import matplotlib.pyplot as plt

# Configuration
MOTION_FILE = "./data/AMASS_EyesJapanDataset/Eyes_Japan_Dataset/aita/jump-11-warmup-aita_poses.npz" # Motion data
XML_PATH = "./models/unitree_h1/scene.xml" # MuJoCo model for the unitree H1 robot (also we can chnage it to G1 robot :))
SCALE_FACTOR = 0.75       # Size adjustment between mocap (AMASS) data and robot
BASE_HEIGHT_OFFSET = 1.0  # Increased vertical offset (height) to prevent floor penetration

# Load motion data
motion_data = np.load(MOTION_FILE)
poses = motion_data["poses"]  # Shape: (frames, 156)
trans = motion_data["trans"]  # Shape: (frames, 3)

# Initialize MuJoCo model
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# Ordered joint mapping with axis correction factors
JOINT_MAPPING = [
    ('pelvis', 'base_link', None),          # Root joint handled separately
    ('left_hip', 'left_hip_pitch', -1),     # Axis inversion
    ('left_knee', 'left_knee', 1),
    ('right_hip', 'right_hip_pitch', -1),
    ('right_knee', 'right_knee', 1),
    ('spine3', 'torso', 1),
    ('left_shoulder', 'left_shoulder_pitch', -1),
    ('left_elbow', 'left_elbow', 1),
    ('right_shoulder', 'right_shoulder_pitch', -1),
    ('right_elbow', 'right_elbow', 1)
]

# Functions
def convert_root_transform(amass_trans, amass_rot):
    """Convert AMASS root transform to MuJoCo Z-up coordinates"""
    # Convert position (AMASS Y-up to MuJoCo Z-up)
    root_pos = np.array([  
        amass_trans[0] * SCALE_FACTOR,     # X remains same
        -amass_trans[2] * SCALE_FACTOR,    # Z becomes Y (with horizontal flip)
        amass_trans[1] * SCALE_FACTOR + BASE_HEIGHT_OFFSET  # Y becomes Z + offset
    ])
    
    # Convert rotation (AMASS Y-up to MuJoCo Z-up) using axis correction
    original_rot = Rotation.from_rotvec(amass_rot)
    adjusted_rot = original_rot * Rotation.from_euler('x', -90, degrees=True) 
    
    # Convert to MuJoCo quaternion format [w, x, y, z]
    quat = adjusted_rot.as_quat()[[3,0,1,2]]  # Reorder to w-first
    
    return root_pos, quat

def retarget_joint_rotation(amass_aa, axis_correction):
    """Convert AMASS joint rotation (axis-angle) to MuJoCo joint angles"""
    # Convert axis-angle to rotation matrix
    rot = Rotation.from_rotvec(amass_aa)
    
    # Axis correction: flipping z-axis to match MuJoCo system (but adjust this for specific joints if necessary)
    corrected_rot = rot * Rotation.from_euler('z', 180, degrees=True)  
    
    # Convert to Euler angles with XYZ convention
    euler = corrected_rot.as_euler('XYZ', degrees=True)
    
    # Apply axis correction and convert to radians
    return np.radians(euler * axis_correction)

def retarget_frame(frame_idx):
"""Retargets a single frame from AMASS data to MuJoCo's robot model."""

    if frame_idx >= len(poses):
        return   # Prevent out-of-range errors
    
    try:
        human_pose = poses[frame_idx]
        human_trans = trans[frame_idx]

        # Process root (pelvis) transform
        root_rot = human_pose[0:3]
        root_pos, root_quat = convert_root_transform(human_trans, root_rot)
        
        # Apply root transform
        data.qpos[0:3] = root_pos
        data.qpos[3:7] = root_quat

        # Process body joints (skip pelvis [0])
        for joint_idx, (amass_joint, h1_joint, axis_corr) in enumerate(JOINT_MAPPING[1:]):
            # Calculate start index in pose array
            start_idx = 3 + joint_idx * 3
            
            # Extract rotation vector
            aa = human_pose[start_idx:start_idx+3] # Axis-angle representation
            
            # Get MuJoCo joint info
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, h1_joint)
            if joint_id == -1:
                continue  # Skip missing joints
               
            # Convert rotation
            joint_angles = retarget_joint_rotation(aa, axis_corr)
            
            # Apply to MuJoCo
            addr = model.jnt_qposadr[joint_id]
            joint_type = model.jnt_type[joint_id]
            
            if joint_type == mujoco.mjtJoint.mjJNT_HINGE:
                # Use primary axis rotation
                data.qpos[addr] = joint_angles[0] # Apply rotation to hinge joints
            elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
                # Apply all three rotations
                data.qpos[addr:addr+3] = joint_angles # Apply rotation to ball joints

    except Exception as e:
        print(f"Retargeting error: {str(e)}")

# Main simulation loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Ensure gravity is enabled for the simulation
    model.opt.gravity[2] = -9.81  # Earth's gravity
    
    # Set camera view
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    
    frame_idx = 0
    while viewer.is_running():
        start_time = time.time()

        # Update the frame
        retarget_frame(frame_idx)
        mujoco.mj_step(model, data)

        # Update viewer
        viewer.sync()

        # Control playback speed (adjust for faster playback)
        elapsed = time.time() - start_time
        time.sleep(max(0, 0.01 - elapsed))  # Adjust frame_rate for faster playback

        frame_idx = (frame_idx + 1) % len(poses)
        
# Plot layout adjustment (if needed)
plt.tight_layout()
plt.show()
