# robot_arm.py
import pybullet as p
import numpy as np
import time
import math

class RobotArm6DOF:
    def __init__(self, base_pos, base_orn, ee_link_index):
        """
        Initialize the robot arm.
        :param urdf_path: Path to URDF model
        :param base_pos: Base position [x,y,z]
        :param base_orn: Base orientation quaternion [x,y,z,w]
        :param ee_link_index: Index of end-effector link
        """
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", base_pos, base_orn, useFixedBase=True)
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
        self.initial_joint_positions = list(self.current_joint_positions)

    def get_current_position(self):
        """
        Returns the current end-effector position as [x, y, z].
        """
        pos = p.getLinkState(self.robot_id, self.ee_link_index)[4]
        return pos

    def reset_position(self, speed=30, min_speed=1, max_speed=360):
        """
        Reset all joints to their initial positions.
        """
        # Clamp speed
        speed = max(min_speed, min(max_speed, speed))
        success = self.move_to_target_joint(self.initial_joint_positions, speed)
        return success


    def move_to_target_joint(self, joint_angles, speed):
        """
        helper inside robot class
        """

        # Interpolate movement
        joint_diffs = [abs(math.degrees(joint_angles[i] - self.current_joint_positions[i])) for i in self.joint_indices]
        max_diff = max(joint_diffs)
        if max_diff < 1e-6:
            return True  # Already at target

        total_time = max_diff / speed
        steps = max(10, int(total_time / 0.01))
        dt = total_time / steps

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

        self.current_joint_positions = list(joint_angles)
       
        return True
        

    def move_to_position(self, target_pos, speed=30, min_speed=1, max_speed=360):
        """
        Move robot's end-effector to a target position using IK.
        :param target_pos: [x,y,z] target position
        :param speed: degrees per second per joint
        :return: True if successful, False if unreachable
        """
        # Clamp speed
        speed = max(min_speed, min(max_speed, speed))

        # Calculate IK
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_index,
            target_pos,
            lowerLimits=self.joint_lower_limits,
            upperLimits=self.joint_upper_limits,
            jointRanges=[upper - lower for upper, lower in zip(self.joint_upper_limits, self.joint_lower_limits)],
            restPoses=self.current_joint_positions,
            maxNumIterations=200,
            residualThreshold=1e-3
        )

        if joint_angles is None or len(joint_angles) != self.num_joints:
            return False

        # Test reachability without moving
        saved_states = [p.getJointState(self.robot_id, ji)[0] for ji in self.joint_indices]
        for i, ji in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, ji, joint_angles[i])
        achieved_pos = p.getLinkState(self.robot_id, self.ee_link_index)[4]
        for i, ji in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, ji, saved_states[i])

        dist = np.linalg.norm(np.array(achieved_pos) - np.array(target_pos))
        if dist > 0.01:
            return False  # Target unreachable
            
        success = self.move_to_target_joint(joint_angles, speed)
        return success
