import pybullet as p
import pybullet_data
import numpy as np
import time
import xml.etree.ElementTree as ET

class SatelliteSim:
    def __init__(self):
        # Start PyBullet simulator in GUI mode
        self.sim = p.connect(p.GUI)
        p.setGravity(0, 0, 0)  # Disable gravity
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Model loading
        self.satelliteId, self.kukaId, self.num_joints = self._load_models()
        self.target_pos = (1, 0.5, 1) 

        # Target visual marker
        self.target_visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
        self.target_visual_id = p.createMultiBody(baseVisualShapeIndex=self.target_visual_shape_id) 
        p.resetBasePositionAndOrientation(self.target_visual_id, self.target_pos, [0,0,0,1]) 

    def _load_models(self):
        """Handles model loading with more informative error handling"""
        model_paths = [
            ("satellite", "C:\\Users\\Aiden\\OneDrive\\Desktop\\BAZZOCCHI-Research\\bullet3\\data\\cube.urdf"),
            ("robot", "C:\\Users\\Aiden\\OneDrive\\Desktop\\BAZZOCCHI-Research\\bullet3\\data\\kuka_iiwa\\model_free_base.urdf")
        ]

        loaded_models = []
        for model_name, model_path in model_paths:
            try:
                model_id = p.loadURDF(model_path, [0, 0, 0 if model_name == 'satellite' else 0.5], 
                                  p.getQuaternionFromEuler([0, 0, 0]), 
                                  useFixedBase=False)  # Both models are free-floating
                loaded_models.append(model_id)
            except p.error as e:
                print(f"Failed to load {model_name}: {e}")
                return None, None, 0  

        if len(loaded_models) != 2:  # Error if not both models loaded
            return None, None, 0

        # Create fixed joint
        p.createConstraint(loaded_models[0], -1, loaded_models[1], -1, jointType=p.JOINT_FIXED,
                       jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, -0.5],
                       parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]), childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        return loaded_models[0], loaded_models[1], p.getNumJoints(loaded_models[1])

    def get_end_effector_state(self):
        # Enhanced state acquisition with error handling
        try:
            link_state = p.getLinkState(self.kukaId, self.num_joints - 1)  # Assuming last link is end-effector
            end_effector_pos = link_state[0]
            end_effector_orientation = link_state[1]
            return end_effector_pos, end_effector_orientation
        except p.error as e:
            print("Failed to get link state:", e)
            return None, None 


    def get_target_state(self):
        # Similar concept, but for the target state
        return self.target_pos  # Expand if your target has orientation etc.

    def set_joint_states(self, joint_positions):
        print("Received joint positions:", joint_positions)  
        print("Number of joints in the KUKA arm:", p.getNumJoints(self.kukaId))
        
        # Debug: Print desired vs. actual positions
        for i in range(self.num_joints):
            desired_pos = joint_positions[i]
            actual_pos = p.getJointState(self.kukaId, i)[0]
            print(f"Joint {i}: Desired {desired_pos:.3f},  Actual: {actual_pos:.3f}")
        
        try:
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=self.kukaId,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_positions[i]
                )
        except p.error as e:
            print("Failed to set joint states:", e)

    def step(self, action):
        # 1. Apply the action to the simulation
        self.set_joint_states(action)  # Provide the action 
        p.stepSimulation()
        p.resetBasePositionAndOrientation(self.target_visual_id, self.target_pos, [0, 0, 0, 1])

        # 2. Update the simulation 
        #self.sim.step() 

        # 3. Get the new state 
        end_effector_pos, end_effector_orientation = self.get_end_effector_state()
        target_pos = self.get_target_state()
        new_state = np.concatenate((end_effector_pos, end_effector_orientation, target_pos))
        return new_state
 
    def reset(self):
        # Reset the simulation to the initial state
        p.resetSimulation()
        p.setGravity(0, 0, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Reload models
        self.satelliteId, self.kukaId, self.num_joints = self._load_models()
        self.target_pos = (1, 0.5, 1)

        # Reset the target position
        p.resetBasePositionAndOrientation(self.target_visual_id, self.target_pos, [0, 0, 0, 1])

        # Get the initial state
        return self.get_end_effector_state()

    def render(self, mode='human'):
        pass  # Visualization if you choose to implement it

    def close(self):
        p.disconnect()

def extract_joint_limits(urdf_file):
    limits = {}
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    for joint in root.iter('joint'):
        name = joint.attrib['name']
        limit_tag = joint.find('limit')
        if limit_tag is not None:
            limits[name] = (float(limit_tag.attrib['lower']), float(limit_tag.attrib['upper']))

    return limits

# pause for 5 seconds
time.sleep(5)

if __name__ == "__main__":
    sim = SatelliteSim()

    print("Satellite ID:", sim.satelliteId)
    print("KUKA Arm ID:", sim.kukaId)
    if sim.satelliteId is None or sim.kukaId is None:
        print("Model loading failed. Check paths and file integrity.")
    else:
        print("Models loaded successfully")

    # Initial end-effector state
    end_effector_pos, end_effector_orientation = sim.get_end_effector_state()
    print("Initial end-effector position:", end_effector_pos)
    print("Initial end-effector orientation:", end_effector_orientation)

    # Move the arm
    sim.set_joint_states([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])  # Sample joint positions
    sim.step([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])  # Advance the simulation

    # Updated end-effector state
    end_effector_pos, end_effector_orientation = sim.get_end_effector_state()
    print("New end-effector position:", end_effector_pos)
    print("New end-effector orientation:", end_effector_orientation)

    joint_limits = extract_joint_limits("C:\\Users\\Aiden\\OneDrive\\Desktop\\BAZZOCCHI-Research\\bullet3\\data\\kuka_iiwa\\model_free_base.urdf")
    # Movement Loop
    for _ in range(1000):  
        current_positions = [p.getJointState(sim.kukaId, i)[0] for i in range(sim.num_joints)]  

        new_positions = []
        for i, current_pos in enumerate(current_positions):
            joint_name = f'lbr_iiwa_joint_{i + 1}'  # Assumes your joints are named this way
            min_limit, max_limit = joint_limits[joint_name]

            amplitude = 0.7  
            target_pos = current_pos + amplitude * np.sin(time.time())
            new_positions.append(max(min(target_pos, max_limit), min_limit))  

        sim.set_joint_states(new_positions) 
        sim.step(new_positions)

        print("---- Simulation Step -----")  # Mark each simulation step
        print(sim.get_end_effector_state())  # Print the state for analysis

        time.sleep(0.05) 

 

