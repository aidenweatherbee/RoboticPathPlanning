import gym 
import numpy as np
import pybullet as p
from satellite_sim import SatelliteSim

class SatelliteEnv(gym.Env):
    """
    Custom OpenAI Gym environment for zero-gravity path planning with PyBullet.
    """

    def __init__(self):
        super().__init__() 
        self.sim = SatelliteSim()  # Instantiate your simulation class

        # Define action and observation spaces (modify as needed)
        num_joints = self.sim.num_joints
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(num_joints,), dtype=np.float32)  
        # Example observation space - expand based on your sensor inputs 
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32)  

        # Create target visual marker
        self.target_visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
        self.target_visual_id = p.createMultiBody(baseVisualShapeIndex=self.target_visual_shape_id) 

    def step(self, action):
        """
        Takes a step in the simulation, applying the action and computing the next state, reward, etc.
        """
        self.sim.set_joint_states(action)
        self.sim.step(action)

        new_state = self._get_observation()
        reward = self._compute_reward(new_state)
        done = self._check_done()  # Define termination conditions
        info = {}  # Additional info for debugging

        # Get the target position from the simulation
        target_pos = self.sim.get_target_state()

        # Update the target visual marker position
        p.resetBasePositionAndOrientation(self.target_visual_id, target_pos, [0, 0, 0, 1])
        

        return new_state, reward, done, info

    def reset(self):
        """
        Resets the simulation to its initial state and returns the corresponding observation.
        """
        self.sim.reset()
        initial_state = self._get_observation()

        # Get the target position from the simulation
        target_pos = self.sim.get_target_state()

        # Update the target visual marker position
        p.resetBasePositionAndOrientation(self.target_visual_id, target_pos, [0, 0, 0, 1])

        return initial_state

    def close(self):
        """
        Clean up resources at the end of an episode
        """
        self.sim.close()

    def _get_observation(self):
        """
        Gets the current observation from the simulation (end-effector position, target, etc.).
        """
        end_effector_pos, end_effector_orientation = self.sim.get_end_effector_state()
        target_pos = self.sim.get_target_state()

        # You might also want to add end-effector velocities, target velocities, and other relevant data
        return np.concatenate((end_effector_pos, end_effector_orientation, target_pos))

    def _compute_reward(self, state):
        """
        Calculates the reward based on the current state.
        """
        # Implement your reward function here that is created to encourage the agent to reach the goal position. 
        
        # Example: Negative L2 distance between end-effector and target
        reward = -np.linalg.norm(state[:3] - state[-3:])

        return reward

    def _check_done(self):
        """
        Checks if a terminal condition is met (e.g., goal reached, collision, max timesteps).
        """

        # Implement your done conditions here. Example:
        end_effector_pos, _ = self.sim.get_end_effector_state()
        target_pos = self.sim.get_target_state()
        distance_to_target = np.linalg.norm(np.array(end_effector_pos) - np.array(target_pos))
        if distance_to_target < 0.1:  # Check if within a tolerance 
            return True
        return False
    
def test_environment():
    """
    Test that the agent will reach the target position in a few thousand runs.
    """
    
    env = SatelliteEnv()
    num_episodes = 1000
    num_steps = 1000
    for episode in range(num_episodes):
        state = env.reset()
        for step in range(num_steps):
            action = env.action_space.sample()
            next_state, reward, done, info = env.step(action)
            
            # Get the target position from the simulation
            target_pos = env.sim.get_target_state()

            # Update the target visual marker position
            p.resetBasePositionAndOrientation(env.target_visual_id, target_pos, [0, 0, 0, 1])
            
            if done:
                print(f"Episode {episode} finished after {step} steps")
                break
    env.close()

if __name__ == "__main__":
    test_environment()
