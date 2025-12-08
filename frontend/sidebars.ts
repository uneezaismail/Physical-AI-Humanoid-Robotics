import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Physical AI & Humanoid Robotics textbook sidebar
  tutorialSidebar: [
    {
      type: "category",
      label: "Part I: Foundations & Lab",
      collapsible: true,
      collapsed: false, // Part I expanded by default (foundational content)
      items: [
        {
          type: "doc",
          id: "part-1-foundations-lab/chapter-01-embodied-ai",
          label: "Chapter 1: Embodied AI - Where Intelligence Meets the Physical World",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-1-foundations-lab/chapter-02-hardware-setup",
          label: "Chapter 2: Hardware Setup - Building Your Physical AI Foundation",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-1-foundations-lab/chapter-03-physical-ai-architecture",
          label: "Chapter 3: Physical AI Architecture - The Three-Tier System",
          className: "sidebar-chapter-link",
        },
      ],
    },
    {
      type: "category",
      label: "Part II: Robotic Nervous System",
      collapsible: true,
      collapsed: false, // Part II expanded by default (foundational content)
      items: [
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-04-ros2-architecture",
          label: "Chapter 4: ROS2 Architecture - The Nervous System of Physical AI",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-05-nodes-topics-services",
          label: "Chapter 5: Nodes, Topics, Services - The Building Blocks of ROS2",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-06-python-rclpy",
          label: "Chapter 6: Python rclpy - Building ROS2 Nodes in Python",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-07-urdf-humanoids",
          label: "Chapter 7: URDF Humanoids - Describing Robots with Unified Robot Description Format",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-08-launch-parameters",
          label: "Chapter 8: Launch Parameters - Configuring and Launching ROS2 Systems",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-2-robotic-nervous-system/chapter-09-first-ros2-package",
          label: "Chapter 9: First ROS2 Package - Creating Complete ROS2 Applications",
          className: "sidebar-chapter-link",
        },
      ],
    },
    {
      type: "category",
      label: "Part III: Digital Twin",
      collapsible: true,
      collapsed: false, // Part III expanded by default
      items: [
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-10-physics-simulation-intro",
          label: "Chapter 10: Physics Simulation Intro - Bridging Digital and Physical Worlds",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-11-gazebo-setup",
          label: "Chapter 11: Gazebo Setup - Creating Simulation Environments for Robotics",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-12-urdf-sdf-formats",
          label: "Chapter 12: URDF-SDF Formats - Converting Robot Models for Simulation",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-13-sensor-simulation",
          label: "Chapter 13: Sensor Simulation - Digital Senses for Physical AI",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-14-unity-visualization",
          label: "Chapter 14: Unity Visualization - Immersive Digital Twins with Unity 3D",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-3-digital-twin/chapter-15-realistic-environments",
          label: "Chapter 15: Realistic Environments - Creating Authentic Simulation Worlds",
          className: "sidebar-chapter-link",
        },
      ],
    },
    {
      type: "category",
      label: "Part IV: AI Robot Brain",
      collapsible: true,
      collapsed: false, // Part IV expanded by default
      items: [
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-16-isaac-overview",
          label: "Chapter 16: Isaac Overview - NVIDIA's AI Robotics Platform",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-17-isaac-sim",
          label: "Chapter 17: Isaac Sim - High-Fidelity Physics Simulation",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-18-isaac-ros-vslam",
          label: "Chapter 18: Isaac ROS vSLAM - GPU-Accelerated Visual SLAM",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-19-nav2-for-humanoid-navigation",
          label: "Chapter 19: Nav2 for Humanoid Navigation - Specialized Path Planning for Bipedal Robots",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-20-humanoid-kinematics",
          label: "Chapter 20: Humanoid Kinematics - Forward and Inverse Kinematics for Bipedal Robots",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-21-bipedal-locomotion",
          label: "Chapter 21: Bipedal Locomotion - Control Systems for Stable Walking",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-22-manipulation-grasping",
          label: "Chapter 22: Manipulation and Grasping - Robotic Interaction with Objects",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-4-ai-robot-brain/chapter-23-sim-to-real",
          label: "Chapter 23: Sim-to-Real Transfer - Bridging the Reality Gap",
          className: "sidebar-chapter-link",
        },
      ],
    },
    {
      type: "category",
      label: "Part V: VLA Capstone",
      collapsible: true,
      collapsed: false, // Part V expanded by default
      items: [
        {
          type: "doc",
          id: "part-5-vla-capstone/chapter-24-vla-intro",
          label: "Chapter 24: VLA Intro - Vision-Language-Action Models for Robotics",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-5-vla-capstone/chapter-25-voice-to-action",
          label: "Chapter 25: Voice to Action - Speech-Driven Robotic Control",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-5-vla-capstone/chapter-26-cognitive-planning",
          label: "Chapter 26: Cognitive Planning - High-Level Reasoning for Complex Tasks",
          className: "sidebar-chapter-link",
        },
        {
          type: "doc",
          id: "part-5-vla-capstone/chapter-27-capstone-project",
          label: "Chapter 27: Capstone Project - Integrating VLA Systems",
          className: "sidebar-chapter-link",
        },
      ],
    },
  ],
};

export default sidebars;
