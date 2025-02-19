## NOTE: This script extracts human motion from a YouTube video using MediaPipe for pose estimation, then retargets the motion to a MuJoCo simulation of the robot.
# NOT DONE YET & NOT TESTED!

import cv2
import numpy as np
import mediapipe as mp
from scipy.spatial.transform import Rotation
import mujoco
import mujoco.viewer
import time
from pytube import YouTube

# Download and extract frames from YouTube video
def download_video(url, save_path='video.mp4'):
    yt = YouTube(url)
    stream = yt.streams.filter(res="360p").first()
    stream.download(filename=save_path)
    return save_path

def extract_frames(video_path):
    cap = cv2.VideoCapture(video_path)
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    return frames

# Pose estimation with MediaPipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

def extract_poses(frames):
    pose_data = []
    for frame in frames:
        results = pose.process(frame)
        if results.pose_landmarks:
            landmarks = [(lm.x, lm.y, lm.z) for lm in results.pose_landmarks.landmark]
            pose_data.append(np.array(landmarks))
    return pose_data

# Convert MediaPipe keypoints to MuJoCo coordinates
def convert_pose_to_mujoco(mediapipe_pose):
    converted = np.array(mediapipe_pose) * 0.8  # Scaling
    return converted

# MuJoCo setup
XML_PATH = "./models/unitree_h1/scene.xml"
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

def retarget_motion(mujoco_pose):
    for i, joint in enumerate(mujoco_pose):
        if i >= model.nq:
            break
        data.qpos[i] = joint
    mujoco.mj_step(model, data)

# Run simulation
video_url = "https://youtu.be/w_g1i6tzNGk?si=pc6M5VksEKFGfiCg" # Human Running vid
video_path = download_video(video_url)
frames = extract_frames(video_path)
poses = extract_poses(frames)
converted_poses = [convert_pose_to_mujoco(p) for p in poses]

with mujoco.viewer.launch_passive(model, data) as viewer:
    frame_idx = 0
    while viewer.is_running():
        start_time = time.time()
        mujoco.mj_resetData(model, data)
        if frame_idx < len(converted_poses):
            retarget_motion(converted_poses[frame_idx])
        mujoco.mj_step(model, data)
        viewer.sync()
        frame_idx = (frame_idx + 1) % len(converted_poses)
        time.sleep(max(0.033 - (time.time() - start_time), 0))
