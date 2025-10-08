# husky_robot_fixed.py
import pybullet as p
import pybullet_data
import numpy as np
import math

class CarRobot:
    def __init__(self, base_pos=[0,0,0.1], base_orn=[0,0,0,1], max_force=50):
        """
        Initialize the Husky robot.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("husky/husky.urdf", basePosition=base_pos, baseOrientation=base_orn)

        # Husky 4-wheel differential drive: left wheels [2,4], right wheels [3,5]
        self.left_wheels = [2, 4]
        self.right_wheels = [3, 5]
        self.max_force = max_force

    def stop(self):
        """Stop all wheels."""
        for w in self.left_wheels + self.right_wheels:
            p.setJointMotorControl2(self.robot_id, w, p.VELOCITY_CONTROL, targetVelocity=0, force=self.max_force)

    def rotate_to_target(self, target_pos, speed):
        """
        Rotate robot in place to face the target position.
        """
        target_pos = np.array(target_pos)
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        pos = np.array(pos)
        delta = target_pos[:2] - pos[:2]
        desired_yaw = math.atan2(delta[1], delta[0])

        _, _, yaw = p.getEulerFromQuaternion(orn)
        yaw_diff = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi

        while abs(yaw_diff) > 0.01:
            direction = 1 if yaw_diff > 0 else -1
            # Left wheels forward, right wheels backward
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=direction*speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=-direction*speed, force=self.max_force)
            p.stepSimulation()
            _, orn = p.getBasePositionAndOrientation(self.robot_id)
            _, _, yaw = p.getEulerFromQuaternion(orn)
            yaw_diff = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi

        self.stop()

    def drive_straight_to_target(self, target_pos, speed):
        """
        Drive straight toward target after rotation.
        Uses clamped proportional steering and minimum forward speed to avoid backward motion.
        """
        target_pos = np.array(target_pos)
        threshold = 0.05

        while True:
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            pos = np.array(pos)
            delta = target_pos[:2] - pos[:2]
            distance = np.linalg.norm(delta)
            if distance < threshold:
                break

            # Desired heading
            desired_yaw = math.atan2(delta[1], delta[0])
            _, _, yaw = p.getEulerFromQuaternion(orn)
            yaw_diff = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi

            # Proportional steering, clamped to ±50% of speed
            k = 0.5
            correction = k * yaw_diff
            correction = max(-0.5, min(0.5, correction))

            left_vel = speed * (1 - correction)
            right_vel = speed * (1 + correction)

            # Ensure minimum forward speed
            min_speed = 0.1 * speed
            left_vel = max(min_speed, left_vel)
            right_vel = max(min_speed, right_vel)

            # Apply wheel velocities
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=left_vel, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=right_vel, force=self.max_force)

            p.stepSimulation()

        self.stop()
        
    def turn_left_90(self, speed=0.3):
        """
        Turn robot 90 degrees to the left in place using wheel velocities.
        """
        _, orn = p.getBasePositionAndOrientation(self.robot_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)
        target_yaw = yaw + math.pi / 2  # 90 degrees in radians

        while True:
            _, orn = p.getBasePositionAndOrientation(self.robot_id)
            _, _, yaw = p.getEulerFromQuaternion(orn)
            yaw_diff = (target_yaw - yaw + math.pi) % (2*math.pi) - math.pi
            if abs(yaw_diff) < 0.01:
                break

            direction = 1  # left turn
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=-direction*speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=direction*speed, force=self.max_force)

            p.stepSimulation()

        self.stop()

    def turn_right_90(self, speed=0.3):
        """
        Turn robot 90 degrees to the right in place using wheel velocities.
        """
        _, orn = p.getBasePositionAndOrientation(self.robot_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)
        target_yaw = yaw - math.pi / 2  # -90 degrees in radians

        while True:
            _, orn = p.getBasePositionAndOrientation(self.robot_id)
            _, _, yaw = p.getEulerFromQuaternion(orn)
            yaw_diff = (target_yaw - yaw + math.pi) % (2*math.pi) - math.pi
            if abs(yaw_diff) < 0.01:
                break

            direction = 1  # right turn
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=direction*speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=-direction*speed, force=self.max_force)

            p.stepSimulation()

        self.stop()


    def turn_toward_target(self, target_pos, speed=0.3):
        """
        Rotate the car in place until it faces the target (x, y) position.
        target_pos: [x, y, z] but z will be ignored
        """
        # Get current car position
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        pos = np.array(pos)
        target_pos = np.array(target_pos)
        
        # Compute desired yaw in the x-y plane
        delta = target_pos[:2] - pos[:2]  # ignore z
        target_yaw = math.atan2(delta[1], delta[0])

        print(f"car is turning", flush=True)
        _, orn = p.getBasePositionAndOrientation(self.robot_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        # Compute shortest angular difference
        yaw_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(yaw_diff) < 0.05:  # threshold for stopping
            print(f"car finished turning", flush=True)
            return

        direction = 1 if yaw_diff > 0 else -1  # choose shortest rotation direction
        while True:
            _, orn = p.getBasePositionAndOrientation(self.robot_id)
            _, _, yaw = p.getEulerFromQuaternion(orn)
    
            # Compute shortest angular difference
            yaw_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_diff) < 0.05:  # threshold for stopping
                break
    
            newdirection = 1 if yaw_diff > 0 else -1  # choose shortest rotation direction
            if newdirection != direction:
                break
    
            # Apply wheel velocities to rotate
            for lw in self.left_wheels:
                p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                        targetVelocity=-direction*speed, force=self.max_force)
            for rw in self.right_wheels:
                p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                        targetVelocity=direction*speed, force=self.max_force)
    
            p.stepSimulation()
    
        self.stop()
        print(f"car finished turning", flush=True)

    def move_forward_to_target(self, target_pos, speed=0.3):
        """
        Move the robot forward toward the target position until it is close
        or it would pass the target based on heading. Prints debug info to terminal.
        """
        threshold = 0.1  # stopping distance
        
        step_count = 0
        print(f"car going stright", flush=True)
        while True:
            step_count += 1
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            pos = np.array(pos)
            _, _, yaw = p.getEulerFromQuaternion(orn)
    
            # Vector to target in XY plane
            delta = target_pos[:2] - pos[:2]
            distance = np.linalg.norm(delta)
    
            # Heading vector
            heading = np.array([math.cos(yaw), math.sin(yaw)])
            projected_distance = np.dot(delta, heading)
    
    
            # Stop if within threshold
            if distance < threshold:
                print(f"stop since arrived", flush=True)
                # Stop wheels
                for lw in self.left_wheels:
                    p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=self.max_force)
                for rw in self.right_wheels:
                    p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=self.max_force)
                break
    
            # Stop if passed target along heading
            if projected_distance <= 0:
                print(f"stop since passed", flush=True)
                for lw in self.left_wheels:
                    p.setJointMotorControl2(self.robot_id, lw, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=self.max_force)
                for rw in self.right_wheels:
                    p.setJointMotorControl2(self.robot_id, rw, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=self.max_force)
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
        print(f"car finished moving", flush=True)

    def move_to_position(self, target_pos, speed=30, min_speed=5, max_speed=90):
        """
        Align robot to target and drive straight.
        API compatible with RobotArm6DOF.
        """

        # testing
        self.turn_toward_target(target_pos)
        self.move_forward_to_target(target_pos)
        return True
