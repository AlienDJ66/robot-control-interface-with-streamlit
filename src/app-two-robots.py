import streamlit as st
import pybullet as p
from robot_arm import RobotArm6DOF
from wheel_car import CarRobot as Car
from simulation import init_simulation, create_target_marker

st.title("Robot Control Panel")

# ------------------------------
# Streamlit session state setup
# ------------------------------
if "robot_type" not in st.session_state:
    st.session_state.robot_type = "6-DOF Arm"
if "sim_initialized" not in st.session_state:
    st.session_state.sim_initialized = False
if "robot" not in st.session_state:
    st.session_state.robot = None
if "target_marker" not in st.session_state:
    st.session_state.target_marker = None

# ------------------------------
# Robot type selection
# ------------------------------
robot_type = st.selectbox(
    "Select Robot Type",
    ["6-DOF Arm", "Wheeled Car"],
    index=0 if st.session_state.robot_type == "6-DOF Arm" else 1
)

# If robot type changed → reset everything
if robot_type != st.session_state.robot_type:
    st.info(f"Switching to {robot_type}... restarting simulation.")
    st.session_state.clear()
    st.session_state.robot_type = robot_type
    st.session_state.sim_initialized = False

    try:
        st.rerun()  # Streamlit 1.49+
    except AttributeError:
        st.experimental_rerun()

# ------------------------------
# UI Inputs
# ------------------------------
st.subheader("Target Position")
col1, col2, col3 = st.columns(3)
with col1:
    target_x = st.number_input("Target X", value=0.5, step=0.01)
with col2:
    target_y = st.number_input("Target Y", value=0.0, step=0.01)
with col3:
    target_z = st.number_input("Target Z", value=0.5, step=0.01)

speed = st.slider("Speed (deg/s or m/s)", min_value=0, max_value=360, value=30, step=1)
go_button = st.button("Go to Position")

# ------------------------------
# Initialize simulation
# ------------------------------
if not st.session_state.sim_initialized:
    init_simulation()

    if st.session_state.robot_type == "6-DOF Arm":
        st.session_state.robot = RobotArm6DOF(
            base_pos=[0, 0, 0],
            base_orn=p.getQuaternionFromEuler([0, 0, 0]),
            ee_link_index=6
        )
    else:
        st.session_state.robot = Car(
            base_pos=[0, 0, 0],
            base_orn=p.getQuaternionFromEuler([0, 0, 0])
        )

    st.session_state.target_marker = create_target_marker([target_x, target_y, target_z])
    st.session_state.sim_initialized = True

# ------------------------------
# Update target marker
# ------------------------------
p.resetBasePositionAndOrientation(
    st.session_state.target_marker,
    [target_x, target_y, target_z],
    [0, 0, 0, 1]
)

# ------------------------------
# Move robot
# ------------------------------
if go_button:
    success = st.session_state.robot.move_to_position([target_x, target_y, target_z], speed=speed)
    if success:
        # Get actual robot position
        pos, _ = p.getBasePositionAndOrientation(st.session_state.robot.robot_id)
        st.success(f"Moved to actual position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    else:
        st.warning("Cannot reach the target position!")
