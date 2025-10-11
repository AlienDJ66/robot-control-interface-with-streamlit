# wheel_car.py
import pybullet as p
import pybullet_data
import numpy as np
import math


class CarRobot:
    def __init__(self, base_pos=[0, 0, 0.1], base_orn=[0, 0, 0, 1], max_force=50):
        """
        Initialize the Husky robot.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("husky/husky.urdf", basePosition=base_pos, baseOrientation=base_orn)

        # Husky 4-wheel differential drive: left wheels [2,4], right wheels [3,5]
        self.left_wheels = [2, 4]
        self.right_wheels = [3, 5]
        self.max_force = max_force

    def get_current_position(self):
        """
        Returns the current base position as [x, y, z].
        """
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return pos

    def reset_position(self, speed=30):
        """
        Move car to initial position and yaw.
        """
        self.move_to_position([0, 0, 0.1], speed)
        return True
        
    def stop(self):
        """Stop all wheels."""
        for w in self.left_wheels + self.right_wheels:
            p.setJointMotorControl2(
                self.robot_id, w, p.VELOCITY_CONTROL,
                targetVelocity=0, force=self.max_force
            )

    def turn_toward_target(self, target_pos, speed=0.3):
        """
        Rotate the car in place until it faces the target (x, y) position.
        target_pos: [x, y, z] but z will be ignored
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        pos = np.array(pos)
        target_pos = np.array(target_pos)

        # Compute desired yaw in the x-y plane
        delta = target_pos[:2] - pos[:2]
        target_yaw = math.atan2(delta[1], delta[0])

        print("car is turning", flush=True)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        # Compute shortest angular difference
        yaw_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(yaw_diff) < 0.05:
            print("car finished turning", flush=True)
            return

        direction = 1 if yaw_diff > 0 else -1
        while True:
            _, orn = p.getBasePositionAndOrientation(self.robot_id)
            _, _, yaw = p.getEulerFromQuaternion(orn)

            yaw_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_diff) < 0.05:
                break

            new_dir = 1 if yaw_diff > 0 else -1
            if new_dir != direction:
                break

            # Apply wheel velocities to rotate
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=-direction * speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=direction * speed, force=self.max_force)

            p.stepSimulation()

        self.stop()
        print("car finished turning", flush=True)

    def move_forward_to_target(self, target_pos, speed=0.3):
        """
        Move the robot forward toward the target position until it is close
        or it would pass the target based on heading.
        """
        threshold = 0.1
        step_count = 0
        print("car going straight", flush=True)

        while True:
            step_count += 1
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            pos = np.array(pos)
            _, _, yaw = p.getEulerFromQuaternion(orn)

            delta = target_pos[:2] - pos[:2]
            distance = np.linalg.norm(delta)
            if distance < threshold:
                print("stop since arrived", flush=True)
                break

            heading = np.array([math.cos(yaw), math.sin(yaw)])
            projected_distance = np.dot(delta, heading)
            if projected_distance <= 0:
                print("stop since passed", flush=True)
                break

            # Move forward
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=speed, force=self.max_force)

            p.stepSimulation()

        self.stop()
        print("car finished moving", flush=True)

    def move_to_position(self, target_pos, speed=30, min_speed=1, max_speed=30):
        """
        Align robot to target and drive straight.
        
        """
        # Clamp speed
        speed = max(min_speed, min(max_speed, speed))
        speed = speed / 100.0
        self.turn_toward_target(target_pos, speed)
        self.move_forward_to_target(target_pos, speed)
        
        return True
