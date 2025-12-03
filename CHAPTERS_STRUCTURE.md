01-part-1-foundations-lab/
  │   │
  │   ├── 01-chapter-1-embodied-ai/
  │   │   ├── 00-intro.mdx                             # Learning objectives + hook
  │   │   ├── 01-digital-vs-physical-ai.mdx            # Conceptual comparison
  │   │   ├── 02-brain-in-box-vs-body.mdx              # Mermaid diagram + explanation
  │   │   ├── 03-partner-economy.mdx                   # 3 real-world examples
  │   │   ├── 04-why-it-matters.mdx                    # Applications
  │   │   ├── 05-exercises.mdx                         # Thought experiments
  │   │   └── 06-summary.mdx                           # Recap + preview Chapter 2
  │   │
  │   ├── 02-chapter-2-hardware-setup/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-hardware-mandate.mdx                  # ⚠️ Danger admonition
  │   │   ├── 02-workstation-specs.mdx                 # RTX GPU, Ubuntu
  │   │   ├── 03-edge-compute.mdx                      # Jetson Orin Nano
  │   │   ├── 04-sensor-stack.mdx                      # RealSense D435i
  │   │   ├── 05-verification.mdx                      # Checklist
  │   │   └── 06-summary.mdx
  │   │
  │   └── 03-chapter-3-physical-ai-architecture/
  │       ├── 00-intro.mdx
  │       ├── 01-three-tier-architecture.mdx           # Workstation/Edge/Robot
  │       ├── 02-sim-to-real-workflow.mdx              # Mermaid diagram
  │       ├── 03-data-flow.mdx                         # Sensors → Compute → Actuators
  │       ├── 04-lab-setup.mdx                         # Code: Network setup
  │       └── 05-summary.mdx
  │
  ├── 02-part-2-robotic-nervous-system/
  │   │
  │   ├── 04-chapter-4-ros2-architecture/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-dds-middleware.mdx                    # Publish-subscribe pattern
  │   │   ├── 02-ros1-vs-ros2.mdx                      # Why ROS 2 for Physical AI
  │   │   ├── 03-core-components.mdx                   # Nodes, Topics, Services, Actions
  │   │   ├── 04-installation.mdx                      # Code: Install ROS 2 Humble
  │   │   ├── 05-first-demo.mdx                        # Code: Turtlesim
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 05-chapter-5-nodes-topics-services/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-nodes.mdx                             # What is a node?
  │   │   ├── 02-topics.mdx                            # Pub/Sub pattern
  │   │   ├── 03-services.mdx                          # Request/Reply pattern
  │   │   ├── 04-qos-profiles.mdx                      # Reliability, Durability
  │   │   ├── 05-code-publisher.mdx                    # Code: Publisher node
  │   │   ├── 06-code-subscriber.mdx                   # Code: Subscriber node
  │   │   ├── 07-code-service.mdx                      # Code: Service server
  │   │   ├── 08-exercises.mdx
  │   │   └── 09-summary.mdx
  │   │
  │   ├── 06-chapter-6-python-rclpy/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-rclpy-basics.mdx                      # Node lifecycle
  │   │   ├── 02-timer-callbacks.mdx                   # 100 Hz control loops
  │   │   ├── 03-parameters.mdx                        # Dynamic reconfiguration
  │   │   ├── 04-complete-node.mdx                     # Code: Full template
  │   │   ├── 05-hardware-integration.mdx              # Code: RealSense IMU node
  │   │   ├── 06-exercises.mdx
  │   │   └── 07-summary.mdx
  │   │
  │   ├── 07-chapter-7-urdf-humanoids/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-urdf-basics.mdx                       # Links, Joints, Frames
  │   │   ├── 02-humanoid-kinematics.mdx               # Hip, Knee, Ankle joints
  │   │   ├── 03-inertial-properties.mdx               # Mass, CoM, Inertia
  │   │   ├── 04-code-simple-leg.mdx                   # Code: URDF for leg
  │   │   ├── 05-rviz-visualization.mdx                # Code: Load URDF in RViz2
  │   │   ├── 06-exercises.mdx
  │   │   └── 07-summary.mdx
  │   │
  │   ├── 08-chapter-8-launch-parameters/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-python-launch-files.mdx               # Why Python > XML
  │   │   ├── 02-parameter-yaml.mdx                    # YAML configuration
  │   │   ├── 03-multi-node-launch.mdx                 # Code: Launch Gazebo + RViz2
  │   │   ├── 04-exercises.mdx
  │   │   └── 05-summary.mdx
  │   │
  │   └── 09-chapter-9-first-ros2-package/
  │       ├── 00-intro.mdx
  │       ├── 01-workspace-structure.mdx               # src/, build/, install/
  │       ├── 02-package-xml.mdx                       # Dependencies
  │       ├── 03-setup-py.mdx                          # Entry points
  │       ├── 04-code-example-package.mdx              # Code: Complete package
  │       ├── 05-build-and-run.mdx                     # Code: colcon build
  │       ├── 06-exercises.mdx
  │       └── 07-summary.mdx
  │
  ├── 03-part-3-digital-twin/
  │   │
  │   ├── 10-chapter-10-physics-simulation-intro/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-physics-engines.mdx                   # ODE, Bullet, PhysX
  │   │   ├── 02-sim-to-real-gap.mdx                   # Reality gap sources
  │   │   ├── 03-tool-comparison.mdx                   # Gazebo vs Unity vs Isaac Sim
  │   │   └── 04-summary.mdx
  │   │
  │   ├── 11-chapter-11-gazebo-setup/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-installation.mdx                      # Code: Install Gazebo Harmonic
  │   │   ├── 02-world-files.mdx                       # SDF world structure
  │   │   ├── 03-first-world.mdx                       # Code: Simple world
  │   │   ├── 04-ros2-integration.mdx                  # Gazebo-ROS2 bridge
  │   │   ├── 05-exercises.mdx
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 12-chapter-12-urdf-sdf-formats/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-urdf-vs-sdf.mdx                       # Differences, when to use which
  │   │   ├── 02-mesh-preparation.mdx                  # Blender to DAE/OBJ
  │   │   ├── 03-xacro-macros.mdx                      # Simplifying URDFs
  │   │   ├── 04-sdf-world-building.mdx                # Creating environments
  │   │   ├── 05-exercises.mdx
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 13-chapter-13-sensor-simulation/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-camera-simulation.mdx                 # RGB-D in Gazebo
  │   │   ├── 02-lidar-simulation.mdx                  # Ray tracing sensors
  │   │   ├── 03-imu-gps-noise.mdx                     # Adding Gaussian noise
  │   │   ├── 04-ros2-plugin-integration.mdx           # Bridging sensor data
  │   │   ├── 05-exercises.mdx
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 14-chapter-14-unity-visualization/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-unity-ros2-tcp-connector.mdx          # Setup connection
  │   │   ├── 02-importing-urdf-to-unity.mdx           # Robot import
  │   │   ├── 03-visualizing-sensor-data.mdx           # Point clouds in Unity
  │   │   ├── 04-digital-twin-setup.mdx                # Environment setup
  │   │   └── 05-summary.mdx
  │   │
  │   └── 15-chapter-15-realistic-environments/
  │       ├── 00-intro.mdx
  │       ├── 01-building-worlds.mdx                   # Gazebo Building Editor
  │       ├── 02-importing-assets.mdx                  # Fuel / Sketchfab
  │       ├── 03-physics-materials.mdx                 # Friction, bounce
  │       └── 04-summary.mdx
  │
  ├── 04-part-4-ai-robot-brain/
  │   │
  │   ├── 16-chapter-16-isaac-overview/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-omniverse-architecture.mdx            # USD, Nucleus, Kit
  │   │   ├── 02-isaac-sim-vs-ros.mdx                  # Comparison
  │   │   ├── 03-hardware-requirements.mdx             # RTX specifics
  │   │   ├── 04-installation-guide.mdx                # Setup guide
  │   │   └── 05-summary.mdx
  │   │
  │   ├── 17-chapter-17-isaac-sim/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-gui-navigation.mdx                    # Interface tour
  │   │   ├── 02-loading-robots.mdx                    # Importing URDF/USD
  │   │   ├── 03-physics-inspector.mdx                 # Debugging physics
  │   │   ├── 04-python-api-scripting.mdx              # OmniGraph
  │   │   ├── 05-ros2-bridge-setup.mdx                 # Connection to ROS 2
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 18-chapter-18-isaac-ros-vslam/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-vslam-concepts.mdx                    # Visual SLAM theory
  │   │   ├── 02-isaac-ros-visual-slam.mdx             # Node configuration
  │   │   ├── 03-realsense-integration.mdx             # Hardware setup
  │   │   ├── 04-mapping-and-localization.mdx          # Creating maps
  │   │   ├── 05-lab-vslam-navigation.mdx              # Hands-on lab
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 19-chapter-19-nav2-bipedal/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-nav2-architecture.mdx                 # Navigation stack
  │   │   ├── 02-costmaps-for-walking.mdx              # Footprint configuration
  │   │   ├── 03-path-planning-algorithms.mdx          # SmacPlanner
  │   │   ├── 04-controller-server.mdx                 # MPPI controller
  │   │   ├── 05-lab-point-to-point.mdx                # Navigation lab
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 20-chapter-20-humanoid-kinematics/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-forward-kinematics.mdx                # Joint angles to pose
  │   │   ├── 02-inverse-kinematics.mdx                # Pose to joint angles
  │   │   ├── 03-center-of-mass.mdx                    # Stability analysis
  │   │   ├── 04-zmp-criterion.mdx                     # Zero Moment Point
  │   │   ├── 05-lab-ik-solver.mdx                     # Coding an IK solver
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 21-chapter-21-bipedal-locomotion/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-gait-cycles.mdx                       # Walking phases
  │   │   ├── 02-lqr-control.mdx                       # Linear Quadratic Regulator
  │   │   ├── 03-reinforcement-learning-approaches.mdx # Isaac Gym
  │   │   ├── 04-training-a-policy.mdx                 # Sim-to-Real RL
  │   │   ├── 05-lab-walking-robot.mdx                 # Deployment lab
  │   │   └── 06-summary.mdx
  │   │
  │   ├── 22-chapter-22-manipulation-grasping/
  │   │   ├── 00-intro.mdx
  │   │   ├── 01-manipulation-pipeline.mdx             # MoveIt 2
  │   │   ├── 02-grasp-generation.mdx                  # Grasp synthesis
  │   │   ├── 03-visual-servoing.mdx                   # Camera-guided control
  │   │   ├── 04-pick-and-place-lab.mdx                # Manipulation lab
  │   │   └── 05-summary.mdx
  │   │
  │   └── 23-chapter-23-sim-to-real/
  │       ├── 00-intro.mdx
  │       ├── 01-domain-randomization.mdx              # Visual & Physics
  │       ├── 02-system-identification.mdx             # Parameter tuning
  │       ├── 03-deployment-pipeline.mdx               # Docker/Jetson
  │       ├── 04-safety-watchdogs.mdx                  # Emergency stops
  │       └── 05-summary.mdx
  │
  └── 05-part-5-vla-capstone/
      │
      ├── 24-chapter-24-vla-intro/
      │   ├── 00-intro.mdx
      │   ├── 01-what-is-vla.mdx                       # Vision-Language-Action
      │   ├── 02-foundation-models.mdx                 # CLIP, RT-1, RT-2
      │   ├── 03-architecture-overview.mdx             # System integration
      │   └── 04-summary.mdx
      │
      ├── 25-chapter-25-voice-to-action/
      │   ├── 00-intro.mdx
      │   ├── 01-whisper-integration.mdx               # Speech-to-Text
      │   ├── 02-intent-recognition.mdx                # LLM parsing
      │   ├── 03-mapping-to-ros2-actions.mdx           # Command execution
      │   ├── 04-lab-voice-control.mdx                 # Voice lab
      │   └── 05-summary.mdx
      │
      ├── 26-chapter-26-cognitive-planning/
      │   ├── 00-intro.mdx
      │   ├── 01-llm-as-planner.mdx                    # Reasoning agents
      │   ├── 02-prompt-engineering-for-robots.mdx     # Context & Constraints
      │   ├── 03-feedback-loops.mdx                    # Re-planning on failure
      │   ├── 04-lab-clean-the-room.mdx                # Planning lab
      │   └── 05-summary.mdx
      │
      ├── 27-chapter-27-multimodal-interaction/
      │   ├── 00-intro.mdx
      │   ├── 01-vision-language-fusion.mdx            # Integrating modalities
      │   ├── 02-gesture-recognition.mdx               # Visual interaction
      │   ├── 03-human-robot-collaboration.mdx         # Working together
      │   ├── 04-final-integration-lab.mdx             # Capstone prep
      │   └── 05-summary.mdx
      │
      └── 28-chapter-28-capstone-project/
          ├── 00-intro.mdx
          ├── 01-requirements.mdx                      # Capstone specs
          ├── 02-architecture.mdx                      # System design
          ├── 03-week1-simulation.mdx                  # Milestone 1
          ├── 04-week2-hardware.mdx                    # Milestone 2
          ├── 05-week3-presentation.mdx                # Final demo
          └── 06-grading-rubric.mdx
