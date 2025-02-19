import mujoco
from mujoco.viewer import launch

model = mujoco.MjModel.from_xml_path("./models/unitree_h1/h1.xml") # Can also be scene.xml
data = mujoco.MjData(model)

# Disable gravity to see if the robot stays in place:
model.opt.gravity[:] = [0, 0, 0]

viewer = launch(model, data)

# Adjust the camera settings to focuson the robot
viewer.cam.lookat = [0, 0, 1]  # robot's default pose
viewer.cam.distance = 3 # Zoom out 
viewer.cam.elevation = 20 # Slightly above
viewer.cam.azimuth = 90 # Side view

# Run the simulation
while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()

viewer.close()
