# **Sport Motion Retargeting**  

## **Overview**  
This project focuses on motion retargeting using **MuJoCo** and **AMASS motion capture data**. The goal is to make robots perform dynamic and natural human-like movements by transferring human motion data to a robotic model (Unitree H1) and applying advanced techniques such as Inverse Kinematics (IK) and potentially Reinforcement Learning (RL) for real-time adaptation.  

🚧**STATUS: Work in Progress**

---
## Motion Capture Data
💥 **IMPORTANT: Please download the motion capture data from this link**:
[Download Data.zip from OneDrive](https://mbzuaiac-my.sharepoint.com/:u:/g/personal/noora_alhajeri_mbzuai_ac_ae1/EThi8eRnjjJMv9jtxGnR5VwB27gOKf1keogjP0VDRkN9iA?e=iJF5h8)

NOTE: 
**The motion data used in this project comes from the [AMASS dataset](https://amass.is.tue.mpg.de/)**.  

---

## **Project Structure**  
```
├── models/              # MuJoCo robot models (H1, G1, etc.)
├── data/                # Motion capture data (AMASS dataset)
├── scripts/             # Python scripts
├── requirements.txt     # Dependencies for running the project
├── README.md            # Project documentation (you’re reading it right now! )
```
---

## **Installation & Setup**  

### **1. Clone the Repository**  
```bash
git clone https://github.com/Noora-Alhajeri/sport-motion-retargeting.git
cd sport-motion-retargeting
```

### **2. Set Up a Virtual Environment (Recommended)**  
```bash
python -m venv mujoco_env
source mujoco_env/bin/activate  # On Windows: mujoco_env\Scripts\activate
```

### **3. Install Dependencies**  
```bash
pip install -r requirements.txt
```

### **4. Install MuJoCo (if not installed)**  
- Follow the instructions here: [MuJoCo Installation](https://mujoco.readthedocs.io/en/latest/)
- Make sure `MUJOCO_PATH` is set correctly.

---

## **Usage**  
### **Inspect Motion Data**  
Run this to check the structure of an AMASS .npz motion file:
```bash
python scripts/inspect_data.py
```

### **Test MuJoCo Simulation** 
Check if the Unitree H1 model loads properly in MuJoCo:
```bash
python scripts/mujoco_test.py
```

### **Run Motion Retargeting**  
Finally, run the motion retargeting to simulate the human motion on the robot:
```bash
python scripts/motion_retargeting.py
```
---

Happy Coding! 
