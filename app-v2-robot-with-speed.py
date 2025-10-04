import streamlit as st
import pybullet as p
import pybullet_data
import numpy as np
import time
import math

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

    def move_to_position(self, target_pos, speed=30, min_speed=5, max_speed=90):
        """
        Move to target position with speed control.
        Speed is in degrees per second per joint.
        Will return False if the target is unreachable.
        """
        # Clamp speed
        speed = max(min_speed, min(max_speed, speed))
    
        # Compute IK
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_index,
            target_pos,
            lowerLimits=self.joint_lower_limits,
            upperLimits=self.joint_upper_limits,
            jointRanges=[upper - lower for lower, upper in zip(self.joint_lower_limits, self.joint_upper_limits)],
            restPoses=self.current_joint_positions,
            maxNumIterations=200,
            residualThreshold=1e-3
        )
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return False  # IK failed
    
        # ------------------------------
        # Reachability check (non-destructive)
        # ------------------------------
        # Save current joint states
        saved_states = [p.getJointState(self.robot_id, ji)[0] for ji in self.joint_indices]
    
        # Apply IK test solution
        for i, ji in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, ji, joint_angles[i])
    
        # Get EE position from this IK solution
        achieved_pos = p.getLinkState(self.robot_id, self.ee_link_index)[4]
    
        # Restore robot state (so it doesn't visually move)
        for i, ji in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, ji, saved_states[i])
    
        # Measure error
        dist = ((achieved_pos[0] - target_pos[0])**2 +
                (achieved_pos[1] - target_pos[1])**2 +
                (achieved_pos[2] - target_pos[2])**2) ** 0.5
    
        if dist > 0.01:  # tolerance in meters
            return False  # Target is unreachable
    
        # ------------------------------
        # If reachable → interpolate with speed
        # ------------------------------
        joint_diffs = [abs(math.degrees(joint_angles[i] - self.current_joint_positions[i]))
                       for i in self.joint_indices]
        max_diff = max(joint_diffs)
        if max_diff < 1e-6:
            return True  # Already at target
    
        total_time = max_diff / speed  # seconds
        steps = max(10, int(total_time / 0.01))  # at least 10 steps
        dt = total_time / steps
    
        # Interpolate smoothly over steps
        for step in range(steps):
            alpha = (step + 1) / steps
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
            time.sleep(dt)
    
        # Update current joint positions
        self.current_joint_positions = list(joint_angles)
        return True



# ------------------------------
# Streamlit UI
# ------------------------------
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
target_x = st.number_input("Target X", value=0.5, min_value=-1.0, max_value=1.0, step=0.01)
target_y = st.number_input("Target Y", value=0.0, min_value=-1.0, max_value=1.0, step=0.01)
target_z = st.number_input("Target Z", value=0.5, min_value=-1.0, max_value=1.0, step=0.01)


# Speed control
st.markdown("**Speed Control (degrees per second):**  This sets how many degrees per second each joint will move.")
speed = st.slider("Joint Speed", min_value=0, max_value=360, value=30, step=1)

# Go button
go_button = st.button("Go to Position")

if go_button and st.session_state.robot is not None:
    success = st.session_state.robot.move_to_position([target_x, target_y, target_z], speed=speed)
    if success:
        st.success(f"Moved to position ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) at {speed}°/s")
    else:
        st.error("Failed to move robot (IK solution not found).")


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

    # Target marker: visual-only sphere
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.02,
        rgbaColor=[1, 0, 0, 1]  # red
    )
    st.session_state.target_marker = p.createMultiBody(
        baseMass=0,  # mass=0 → non-physical
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[target_x, target_y, target_z]
    )

    st.session_state.sim_initialized = True
    p.setRealTimeSimulation(1)

# ------------------------------
# Update target marker position
# ------------------------------
p.resetBasePositionAndOrientation(
    st.session_state.target_marker,
    [target_x, target_y, target_z],
    [0, 0, 0, 1]
)

# ------------------------------
# Move to Position when button pressed
# ------------------------------
if go_button:
    success = st.session_state.robot.move_to_position([target_x, target_y, target_z])
    if not success:
        st.warning("Cannot reach the target position!")
