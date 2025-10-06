# app.py
import streamlit as st
import pybullet as p
from robot_arm import RobotArm6DOF
from simulation import init_simulation, create_target_marker

st.title("6-DOF Robot Arm Control")

# ------------------------------
# Streamlit session state
# ------------------------------
if "sim_initialized" not in st.session_state:
    st.session_state.sim_initialized = False
if "robot" not in st.session_state:
    st.session_state.robot = None
if "target_marker" not in st.session_state:
    st.session_state.target_marker = None

# ------------------------------
# UI Inputs
# ------------------------------
target_x = st.number_input("Target X", value=0.5, min_value=-1.0, max_value=1.0, step=0.01)
target_y = st.number_input("Target Y", value=0.0, min_value=-1.0, max_value=1.0, step=0.01)
target_z = st.number_input("Target Z", value=0.5, min_value=0.0, max_value=1.0, step=0.01)

speed = st.slider("Joint Speed (deg/s)", min_value=0, max_value=360, value=30, step=1)
go_button = st.button("Go to Position")

# ------------------------------
# Initialize simulation and robot
# ------------------------------
if not st.session_state.sim_initialized:
    init_simulation()
    st.session_state.robot = RobotArm6DOF(
        urdf_path="kuka_iiwa/model.urdf",
        base_pos=[0,0,0],
        base_orn=p.getQuaternionFromEuler([0,0,0]),
        ee_link_index=6
    )
    st.session_state.target_marker = create_target_marker([target_x, target_y, target_z])
    st.session_state.sim_initialized = True

# ------------------------------
# Update target marker
# ------------------------------
p.resetBasePositionAndOrientation(
    st.session_state.target_marker,
    [target_x, target_y, target_z],
    [0,0,0,1]
)

# ------------------------------
# Move robot when button pressed
# ------------------------------
if go_button:
    success = st.session_state.robot.move_to_position([target_x, target_y, target_z], speed=speed)
    if success:
        st.success(f"Moved to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
    else:
        st.warning("Cannot reach the target position!")
