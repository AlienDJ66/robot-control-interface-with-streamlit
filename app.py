import streamlit as st
import pybullet as p
import pybullet_data
import numpy as np
import time

# ------------------------------
# Robot Arm Class, this can be made to interface with move to position implemented
# ------------------------------
class RobotArm6DOF:
    def __init__(self, urdf_path, base_pos, base_orn, ee_link_index):
        self.robot_id = p.loadURDF(urdf_path, base_pos, base_orn, useFixedBase=True)
        self.ee_link_index = ee_link_index
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = [i for i in range(self.num_joints)]
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        for i in self.joint_indices:
            info = p.getJointInfo(self.robot_id, i)
            self.joint_lower_limits.append(info[8])
            self.joint_upper_limits.append(info[9])
        self.current_joint_positions = [p.getJointState(self.robot_id, i)[0] for i in self.joint_indices]

    def move_to_position(self, target_pos, steps=100):
        # Compute IK
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_index,
            target_pos,
            lowerLimits=self.joint_lower_limits,
            upperLimits=self.joint_upper_limits,
            jointRanges=[upper-lower for lower, upper in zip(self.joint_lower_limits, self.joint_upper_limits)],
            restPoses=self.current_joint_positions,
            maxNumIterations=200,
            residualThreshold=1e-3
        )
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return False  # IK failed

        # Interpolate smoothly over 'steps'
        for step in range(steps):
            alpha = (step+1)/steps
            for i, ji in enumerate(self.joint_indices):
                target = self.current_joint_positions[i] + alpha * (joint_angles[i] - self.current_joint_positions[i])
                p.setJointMotorControl2(
                    self.robot_id,
                    ji,
                    p.POSITION_CONTROL,
                    targetPosition=target,
                    force=500
                )
            p.stepSimulation()
            time.sleep(0.01)

        # Update current joint positions
        self.current_joint_positions = list(joint_angles)
        return True

# ------------------------------
# Streamlit UI
# ------------------------------
st.title("6-DOF Robot Arm Control")

if "sim_initialized" not in st.session_state:
    st.session_state.sim_initialized = False
if "robot" not in st.session_state:
    st.session_state.robot = None
if "target_marker" not in st.session_state:
    st.session_state.target_marker = None

# Input fields for target position
target_x = st.number_input("Target X", 0.5)
target_y = st.number_input("Target Y", 0.0)
target_z = st.number_input("Target Z", 0.5)

go_button = st.button("Go to Position")

# ------------------------------
# Initialize PyBullet
# ------------------------------
if not st.session_state.sim_initialized:
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # Load KUKA robot arm
    st.session_state.robot = RobotArm6DOF(
        urdf_path="kuka_iiwa/model.urdf",
        base_pos=[0,0,0],
        base_orn=p.getQuaternionFromEuler([0,0,0]),
        ee_link_index=6  # end-effector link
    )

    # Target marker sphere
    st.session_state.target_marker = p.loadURDF(
        "sphere_small.urdf",
        basePosition=[target_x, target_y, target_z],
        useFixedBase=True
    )

    st.session_state.sim_initialized = True
    p.setRealTimeSimulation(1)

# ------------------------------
# Update target marker position
# ------------------------------
p.resetBasePositionAndOrientation(
    st.session_state.target_marker,
    [target_x, target_y, target_z],
    [0,0,0,1]
)

# ------------------------------
# Move to Position when button pressed
# ------------------------------
if go_button:
    success = st.session_state.robot.move_to_position([target_x, target_y, target_z])
    if not success:
        st.warning("Cannot reach the target position!")
