import numpy as np

# Load any motion data
motion_data = np.load("./data/AMASS_EyesJapanDataset/Eyes_Japan_Dataset/aita/jump-11-warmup-aita_poses.npz")

# List all keys in the file to see the structure
print("Keys in the motion data:", motion_data.files)

# Check the shape of the poses and translations
poses = motion_data["poses"]
trans = motion_data["trans"]

print(f"Poses shape: {poses.shape}")
print(f"Trans shape: {trans.shape}")

# Optionally, view a sample pose and translation
print("Sample pose data (first frame):", poses[0])
print("Sample translation data (first frame):", trans[0])
