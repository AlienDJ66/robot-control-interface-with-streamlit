import streamlit as st
import pybullet as p
from robot_arm import RobotArm6DOF
from wheel_car import CarRobot as Car
from simulation import init_simulation, create_target_marker
import time

st.title("Robot Control Panel")

# ------------------------------
# Streamlit session state setup
# ------------------------------
if "robot_type" not in st.session_state:
    st.session_state.robot_type = "6-DOF Arm"
if "sim_running" not in st.session_state:
    st.session_state.sim_running = False
if "robot" not in st.session_state:
    st.session_state.robot = None
if "target_marker" not in st.session_state:
    st.session_state.target_marker = None
if "move_result" not in st.session_state:
    st.session_state.move_result = None

# ------------------------------
# Robot type selection
# ------------------------------
robot_type = st.selectbox(
    "Select Robot Type",
    ["6-DOF Arm", "Wheeled Car"],
    index=0 if st.session_state.robot_type == "6-DOF Arm" else 1
)

# If robot type changed → reset simulation
if robot_type != st.session_state.robot_type:
    st.session_state.robot_type = robot_type
    if st.session_state.sim_running:
        p.disconnect()
        st.session_state.sim_running = False
        st.session_state.robot = None
        st.session_state.target_marker = None
        st.info(f"Robot type changed to {robot_type}, simulation stopped. Click Start Simulation to restart.")

# ------------------------------
# Simulation control buttons
# ------------------------------
col_start, col_stop = st.columns(2)
with col_start:
    start_sim = st.button("Start Simulation")
with col_stop:
    stop_sim = st.button("Stop Simulation")

if stop_sim and st.session_state.sim_running:
    p.disconnect()
    st.session_state.sim_running = False
    st.session_state.robot = None
    st.session_state.target_marker = None
    st.success("Simulation stopped.")

if start_sim and not st.session_state.sim_running:
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
    # Initial target marker
    st.session_state.target_marker = create_target_marker([0.5, 0.0, 0.5])
    st.session_state.sim_running = True
    st.session_state.move_result = None
    st.success("Simulation started.")

# ------------------------------
# Only show controls if simulation is running
# ------------------------------
if st.session_state.sim_running:
    st.subheader("Target Position")
    col1, col2, col3 = st.columns(3)
    with col1:
        target_x = st.number_input("Target X", value=0.5, step=0.05)
    with col2:
        target_y = st.number_input("Target Y", value=0.0, step=0.05)
    with col3:
        target_z = st.number_input("Target Z", value=0.5, step=0.05)

    if st.session_state.robot_type == "6-DOF Arm":
        speed = st.slider("Speed (deg/s)", min_value=0, max_value=360, value=30, step=5)
    else :
        speed = st.slider("Speed ", min_value=0, max_value=30, value=30, step=1)
    go_button = st.button("Go to Position")
    reset_button = st.button("Reset Position")

    # ------------------------------
    # Update target marker
    # ------------------------------
    p.resetBasePositionAndOrientation(
        st.session_state.target_marker,
        [target_x, target_y, target_z],
        [0, 0, 0, 1]
    )

    # ------------------------------
    # Move robot directly (blocking)
    # ------------------------------
    if go_button:
        result = st.session_state.robot.move_to_position([target_x, target_y, target_z], speed=speed)
        st.session_state.move_result = result
    if reset_button:
        if st.session_state.robot is not None:
            result = st.session_state.robot.reset_position(speed)
            st.session_state.move_result = result

    # ------------------------------
    # Show movement result
    # ------------------------------
    if st.session_state.move_result is not None:
        if st.session_state.move_result:  # success
            pos = st.session_state.robot.get_current_position()
            st.success(f"Moved to actual position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        else:
            st.warning("Cannot reach the target position!")
        st.session_state.move_result = None  # reset to show only once

else:
    st.info("Simulation not running. Click 'Start Simulation' to initialize PyBullet.")
