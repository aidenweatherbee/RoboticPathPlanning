Below is a plan for the overview of this project. the project is in the very early stages

Zero-Gravity Reinforcement learning based Path planning and collision avoidance.

Goal: Develop a reinforcement learning system to control a robotic arm in a simulated zero-gravity environment. The objective is for a robotic arm attached to a satellite to successfully reach a given location and orientation on/near another satellite, and maintain this position for long enough for grasping to be performed without any collisions occurring. Initially both satellites will be stationary in a different location within the robotic arms reach for each episode, with a substantially heavier satellite than the robotic arm. Later tests will introduce a different random relative velocity between the two satellites in each episode(towards each other), as well as reduce the relative weight between the robotic arm and the satellite. 

Scope:The first assumption is that the vision code will deliver the x, y, z, quaternions/(roll, pitch, yaw), and their velocities. My initial thoughts are that the frames will not be processed by the reinforcement learning algorithm( I will look into whether providing depth/ lidar frames is something to consider). The second thing is that gripping will not be within the scope of the initial project. The goal of the reinforcement learning algorithm will be to get the end effector to the proper location and orientation for a long enough duration of time to allow gripping. The assumption is that the gripping will be done using separate machine learning techniques that take into account the degrees of freedom of the gripper and other relevant factors that are outside of the scope of path planning. The goal location and orientation will be chosen such that the gripper must have the object on the satellite between the fingers to allow gripping. 

Simulation Environment:

- Framework: PyBullet
- Models:
  - Satellite with robotic arm with relevant degrees of freedom. (decide later)
  - Target satellite object in zero-gravity space.

Reinforcement Learning Setup

- Algorithms considered: 
  - 1st choice: Proximal Police Optimization (PPO)
  - 2nd choice: Soft Actor-Critic (SAC)
  - 3rd choice : Deep Deterministic Policy Gradient (DDPG)
  
- Library: Stable Baseline 3.

State Space

- Components: All components are relative to the end effector.
  - Target x, y, z coordinates
  - Target orientation: roll, pitch, yaw. Or quaternions
  - Velocities for all components mentioned above.
    
-Considerations:
  - Would lidar/depth frames be a good idea to include as well? Which method is more used/ a better idea? Are lidar/depth frames ever used or is it usually split up into vision code and path planning separately(this is what I think)?

Action Space

- Control: Joint torques or joint angles for the robotic arm. More likely joint torques, but which is a better idea? Would joint torques make this significantly more complex? I believe it would significantly increase real world performance when transferring from the simulation but more research is needed.

Reward Function

- Positive Rewards:
  - Proximity to the target location and orientation.
  - Successfully reaching the target location and orientation.
  - Maintaining the position for a duration.

- Penalties:
  - Collisions with other objects in the simulation
  - Penalize excessive time taken
  
- Considerations:
  - Much more research is needed to design the reward function(ex. Should the max reward be 0 like Professor Bazzocchi’s paper) 
  - I will consider adding reward terms for minimal energy consumption and smoothness of the trajectory later.

Training
- Hyperparameters: I will experiment with these:
  - Learning rates (actor and critic)
  - Exploration noise parameters
  - Replay buffer size
  -,Discount factor (gamma)

Additional Considerations

-Observation Normalization: Scale state observations for stability.

-Action Scaling: Ensure joint torques are bounded within realistic ranges.

-Curriculum Learning: Potentially start with the goal of reaching the location and orientation, then introducing the goal of staying in this position/ orientation.

-Hierarchical Reinforcement Learning: This is probably not in the scope for this summer, as it would significantly increase the complexity of the simulation, but I will potentially look into breaking the problem into subgoals. The high level policy sets long term goals( reach target, avoid collision, etc), and lower level policies focus on the quality of the joint movement and similar factors. 

-Transfer from simulation to real: Introduce randomization where necessary by varying parameters to increase the chances of successful sim to real transfer. 

-Metrics: I will track the average reward per episode, collision frequency, success rate, distance from location, etc.

-Configuration: I will consider having a configuration file (JSON, YAML, etc.) to store parameters like simulation settings, agent hyperparameters, and training settings to make experimentation easier.

-Experiment Tracking: For more organized research, I will consider using a basic experiment tracking tool to log different runs, parameters, and the resulting metrics. Need to look into the best ways to do this.

-Historical Data: I will consider adding a short-term memory to the state space. This will incorporate the last few timesteps which might help infer object trajectories better

-Simulation Environment Setup with Docker and PyBuilder

-Docker: Docker will be used to containerize the entire simulation environment, ensuring that the PyBullet framework and Stable Baseline 3 library, along with all necessary dependencies, are encapsulated in a consistent and isolated setting. This will facilitate easy sharing and deployment of the simulation environment across different machines or platforms without the need for complex setup procedures.

-PyBuilder: PyBuilder will serve as the build automation tool for the Python components of the project. It will manage dependencies and streamline the build process, making it easier to compile and test the codebase. PyBuilder will also help in packaging the project for distribution or deployment, ensuring that all Python modules and required libraries are correctly installed. Benefits: Reproducibility: Both Docker and PyBuilder contribute to the reproducibility of the simulation environment, which is crucial for collaborative development and research validation. Consistency: They ensure that the environment remains consistent across different stages of development, testing, and production. Efficiency: Automating the build and deployment process saves time and reduces the potential for human error during setup. Implementation Details: A Dockerfile will be created to define the steps for setting up the simulation environment within a Docker container. A build.py file will be used to specify the PyBuilder configuration, detailing how the Python project should be built and packaged

Project structure: 

1. Core Simulation (satellite_sim.py)
Handles PyBullet interaction, object loading, and physics:
load_objects(self): Load your satellite, KUKA arm models, and any other relevant objects into the PyBullet simulation.
set_initial_positions(self): Initialize starting positions of objects.
get_end_effector_state(self): Retrieve end-effector position, orientation, and possibly velocities.
get_target_state(self): Function to get the target position and orientation.
set_joint_states(self, joint_positions): Apply target joint positions to the robot arm.
step(self): Advance the simulation by a single timestep.
reset(self): Reset the simulation to its initial state.
render(self): Handle visualization, if necessary.
2. Reinforcement Learning Environment (satellite_env.py)
Built on top of satellite_sim.py this inherits gym.Env for RL compatibility
Constructor: Create an instance of satellite_sim.py
step(self, action): Takes an action, applies it in the core simulation, calculates the reward, observes the new state, and determines if a 'done' condition is reached. Returns state, reward, done, info.
reset(self): Uses satellite_sim.reset() and returns the initial state.
calculate_reward(self): Implements your reward function logic.
3. Reinforcement Learning Agent & Training (rl_agent.py)
Handles all RL-specific code:
build_model(self): Create your Stable Baselines3 model (e.g., PPO).
train(self): The main agent training loop, handling model updates, environment interaction, etc.
save_model(self): Save the trained model.
load_model(self): Load a pre-trained model.
4. Configuration (config.py or config.json)
Store experiment parameters:
Simulation parameters (target locations, object dimensions, etc.)
Reinforcement learning hyperparameters (learning rate, discount factor, etc.)
Training parameters (number of timesteps, logging frequency)
5. Experiment Runner (run_experiment.py)
Orchestrates everything:
Loads configuration
Creates the satellite_sim environment
Implements SatelliteEnv (your custom RL environment)
Creates the rl_agent
Runs the training loop

Structure
project_folder/
    config.json
    rl_agent.py
    run_experiment.py
    satellite_env.py
    satellite_sim.py
